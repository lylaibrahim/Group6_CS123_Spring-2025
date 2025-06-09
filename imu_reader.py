"""
High-frequency IMU reader that writes to a shared file for the publisher to read.
"""

import math, time, serial
from serial.tools import list_ports
import numpy as np
import json
import sys
import select

# USER SETTINGS — tweak to taste
BAUDRATE       = 115200
TIMEOUT        = 1.0             # seconds

# thresholds   -------------------------------------------------------------- #
YAW_TRIG_DEG   = 15.0            # |Δyaw| > 10° → left / right
PITCH_TRIG_DEG = 15.0            #  Δpitch  > 10° → forward
ROLL_TRIG_DEG  = 20.0            # |Δroll| > 5° → roll command

# helper: auto-find Teensy's usbmodem* port so you never re-edit the script
def find_teensy_port():
    for p in list_ports.comports():
        if "usbmodem" in p.device.lower():
            return p.device      # e.g. /dev/cu.usbmodem170925201
    raise RuntimeError("Teensy serial port not found.")

PORT = find_teensy_port()
print(f"Using serial port: {PORT}")

def q_conj(q):
    """conjugate (inverse for a unit quaternion)"""
    w,x,y,z = q
    return (w, -x, -y, -z)

def q_mul(a, b):
    """Hamilton product  a ⊗ b"""
    w1,x1,y1,z1 = a
    w2,x2,y2,z2 = b
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Returns angles in degrees
    """
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    # Use 1 if sinp is out of range
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return roll, pitch, yaw

def rotate_vec(q, v):
    """
    Rotate 3-vector v (numpy) by quaternion q.
    v' = q ⊗ (0,v) ⊗ q*
    """
    w,x,y,z = q
    qvec    = np.array([x, y, z])
    uv      = np.cross(qvec, v)
    uuv     = np.cross(qvec, uv)
    return v + 2*(w*uv + uuv)

def get_orientation_deltas(forward, forward_0, q_now, q0):
    """
    Return signed roll, pitch, and yaw deltas (in degrees)
    between the current orientation and the baseline one.
    • roll  – rotation around forward axis
    • pitch – change in vertical tilt (positive = looking up)
    • yaw   – angle in the horizontal (XZ) plane
    """
    if forward is None or forward_0 is None:
        print("Warning: Received None vector in get_orientation_deltas")
        return 0.0, 0.0, 0.0
    
    try:
        # Calculate relative quaternion
        q_rel = q_mul(q_conj(q0), q_now)
        w, x, y, z = q_rel
        
        # Calculate roll directly from quaternion
        # This method uses the quaternion components to calculate roll
        # while being independent of yaw and pitch
        roll = math.degrees(2.0 * math.atan2(x * y + w * z, w * w + x * x - y * y - z * z))
        
        # Convert forward vectors to spherical coordinates for yaw and pitch
        def to_spherical(v):
            try:
                # Ensure v is a numpy array
                v = np.array(v, dtype=float)
                r = np.linalg.norm(v)
                if r == 0:
                    print("Warning: Zero vector in to_spherical")
                    return 0.0, 0.0
                    
                # Calculate angles
                theta = math.atan2(v[0], v[2])  # yaw
                phi = math.asin(np.clip(v[1]/r, -1, 1))  # pitch
                
                return float(theta), float(phi)  # Ensure we return Python floats
            except Exception as e:
                print(f"Error in to_spherical: {str(e)}")
                print(f"Vector v: {v}, type: {type(v)}")
                return 0.0, 0.0
        
        theta0, phi0 = to_spherical(forward_0)
        theta1, phi1 = to_spherical(forward)
        
        # Calculate yaw and pitch differences
        yaw = math.degrees(theta1 - theta0)
        pitch = math.degrees(phi1 - phi0)
        
        # Normalize yaw to [-180, 180]
        yaw = (yaw + 180) % 360 - 180
        
        return -roll, pitch, yaw
    except Exception as e:
        print(f"Error in get_orientation_deltas: {str(e)}")
        print(f"forward_0: {forward_0}, forward: {forward}")
        return 0.0, 0.0, 0.0

def check_for_reset():
    """Check if 'r' key is pressed without blocking"""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        key = sys.stdin.read(1)
        return key.lower() == 'r'
    return False

def main():
    ### IMU logic ###
    q0  = None           # reference quaternion (first good packet)
    f0  = None           # baseline forward vector
    FWD_VEC = np.array([0, 0, -1], dtype=float)  # assumes sensor's "front" = -Z

    # Set up non-blocking keyboard input
    import termios
    import tty
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
            print(f"Listening at {BAUDRATE} baud … (Ctrl-C to quit)")
            print("Press 'r' to reset reference orientation")
            ser.readline()   # throw away any half-line

            while True:
                try:
                    # Check for reset key
                    if check_for_reset():
                        q0 = None
                        f0 = None
                        print("\n↪ Reference orientation reset")
                        continue

                    raw = ser.readline().decode().strip()
                    if not raw:
                        continue
                    parts = raw.split()
                    if len(parts) != 4:
                        print("⚠️ malformed:", raw)
                        continue

                    # ----------------------------------------------------------------------------                
                    q_now = tuple(map(float, parts))
                    
                    if q0 is None:
                        q0 = q_now
                        f0 = rotate_vec(q0, FWD_VEC)
                        print("↪ reference captured; everything now relative to this pose.")
                        continue

                    # Calculate relative orientation
                    q_rel = q_mul(q_conj(q0), q_now)
                    f_now = rotate_vec(q_now, FWD_VEC)

                    # Get orientation deltas
                    roll_delta, pitch_delta, yaw_delta = get_orientation_deltas(f_now, f0, q_now, q0)
                    
                    # Create movement command based on thresholds
                    yaw_command = "0.0"
                    pitch_command = "0.0"
                    roll_command = "0.0"
                    
                    if abs(yaw_delta) > YAW_TRIG_DEG:
                        yaw_value = np.clip(yaw_delta / YAW_TRIG_DEG / 5, -1.5, 1.5)
                        yaw_command = str(round(yaw_value, 3))
                    if abs(pitch_delta) > PITCH_TRIG_DEG:
                        pitch_value = np.clip(pitch_delta / PITCH_TRIG_DEG / 5, -1.0, 1.0)
                        pitch_command = str(round(pitch_value, 3))
                    if abs(roll_delta) > ROLL_TRIG_DEG:
                        roll_value = np.clip(roll_delta / ROLL_TRIG_DEG / 5, -0.5, 0.5)
                        roll_command = str(round(roll_value, 3))
                    
                    command = f"{yaw_command},{pitch_command},{roll_command}"
                    
                    # Print high-frequency IMU updates
                    print(f"IMU: yaw={yaw_delta:6.1f}° pitch={pitch_delta:6.1f}° roll={roll_delta:6.1f}° command={command}")
                    
                    # Write current state to shared file
                    state = {
                        'yaw_delta': float(yaw_delta),
                        'pitch_delta': float(pitch_delta),
                        'roll_delta': float(roll_delta),
                        'command': command,
                        'timestamp': time.time()
                    }
                    with open('imu_state.json', 'w') as f:
                        json.dump(state, f)

                    time.sleep(0.005)  # High-frequency IMU reading

                except (UnicodeDecodeError, ValueError) as e:
                    print("⚠️ parse error:", e, "in", raw)
                except KeyboardInterrupt:
                    print("Stopping.")
                    break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main() 