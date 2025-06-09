#!/usr/bin/env python3

import socket
import time
import json
from datetime import datetime

class UDPPublisher:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self.sock = None
        self.message_count = 0
        self.start_time = time.time()

    def connect(self):
        """Establish UDP connection"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Test connection by sending a ping
            self.sock.sendto(b"ping", (self.hostname, self.port))
            self.sock.settimeout(1.0)  # 1 second timeout for receiving
            try:
                data, addr = self.sock.recvfrom(1024)
                if data == b"pong":
                    print(f"Connected to {self.hostname}:{self.port}")
                    return True
            except socket.timeout:
                print("Warning: No response from server, but continuing anyway")
                return True
        except Exception as e:
            print(f"Connection Error: {str(e)}")
            return False

    def send_message(self, message):
        """Send a message via UDP"""
        if not self.sock:
            print("Not connected")
            return False

        try:
            # Ensure message is a string
            message = str(message)
            
            # Send the message
            self.sock.sendto(message.encode(), (self.hostname, self.port))
            
            self.message_count += 1
            current_time = time.time()
            elapsed = current_time - self.start_time
            print(f"Message {self.message_count} sent at {elapsed:.2f}s: {message}")
            return True
            
        except Exception as e:
            print(f"Error sending message: {str(e)}")
            return False

    def close(self):
        """Close the UDP connection"""
        if self.sock:
            self.sock.close()
            self.sock = None

def read_imu_state():
    """Read the current IMU state from the shared file"""
    try:
        with open('imu_state.json', 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None

def main():
    # Connection details
    hostname = "pupper.local"  # or use IP address
    port = 5000  # Choose a port that's not in use
    
    # Create publisher instance
    publisher = UDPPublisher(hostname, port)
    
    # Connect to the system
    if not publisher.connect():
        print("Failed to connect to UDP server")
        return
    
    print("\nUDP Command Publisher")
    print("Press Ctrl+C to exit")
    
    last_command = None
    last_command_time = 0
    COMMAND_INTERVAL = 0.1  # Send command every 100ms (much faster than before)

    try:
        while True:
            # Read current IMU state
            state = read_imu_state()
            if state is None:
                print("Waiting for IMU data...")
                time.sleep(0.1)
                continue

            current_time = time.time()
            if current_time - last_command_time >= COMMAND_INTERVAL:
                command = state['command']
                print(f"\rCurrent state: yaw={state['yaw_delta']:.1f}° pitch={state['pitch_delta']:.1f}° command={command}", end="")
                
                if not publisher.send_message(command):
                    print("\nFailed to send command, retrying connection...")
                    if not publisher.connect():
                        print("Failed to reconnect, exiting...")
                        break
                
                last_command = command
                last_command_time = current_time

            time.sleep(0.01)  # Check for new commands every 10ms

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        publisher.close()

if __name__ == '__main__':
    main() 