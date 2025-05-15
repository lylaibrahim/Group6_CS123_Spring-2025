import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from openai import OpenAI

client = OpenAI(api_key='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')

import numpy as np
import time
import io
import wave
import whisper as wh

# Set your OpenAI API key here

class CommandLinePublisher(Node):
    def __init__(self):
        super().__init__('command_line_publisher')

        # Create a publisher for the user query topic
        self.publisher_ = self.create_publisher(
            String,
            'user_query_topic',  # Replace with the topic name used in your GPT-4o node
            10
        )
        self.get_logger().info('Command Line Publisher Node has started.')
        self.model = wh.load_model("tiny")

    # TODO: Implement the publish_message method
    # message is a string that contains the user query. You can publish it using the publisher_ and its publish method
    def publish_message(self, message):
        
        # Create a String message and publish it with the message as the data
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

        # DEBUG LOGGER: Uncomment the following line to print the message (you may have to change the variable name)
        # self.get_logger().info(f"Published message: {message}")
            
    def transcribe_audio_with_whisper(self, filename, sample_rate=16000):
        try:
            print("Transcribing audio using Whisper model...")
            audio_file = open(filename, "rb")
            transcription = client.audio.transcriptions.create(
                model="whisper-1", file=audio_file, response_format="text")
            self.get_logger().info(f"Transcription response: {transcription}")
            return transcription
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None



def record_audio(duration=5, sample_rate=16000):
    print("Recording audio...")
    audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
    sd.wait()  # Wait until the recording is finished
    print("Audio recording finished.")
    return np.squeeze(audio_data)

def audio_to_wav(audio_data, sample_rate=16000):
    """Convert numpy array audio to WAV format"""
    wav_io = io.BytesIO()
    with wave.open(wav_io, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_data.tobytes())
    wav_io.seek(0)
    return wav_io

def main(args=None):
    rclpy.init(args=args)

    # Create the command line publisher node
    command_publisher = CommandLinePublisher()

    # Set up the stream and audio processing
    print("Listening for speech every 0.9 seconds. Say 'exit' to stop.")

    command_publisher.get_logger().info("Starting recording.")

    try:
        audio_data = record_audio(duration=5.0)
        # Save the recorded audio to a WAV file
        wav_io = audio_to_wav(audio_data)
        filename = '/home/pi/pupper_llm/pupper_llm/simple_scripts/test_audio.wav'
        with open(filename, 'wb') as f:
            f.write(wav_io.read())
        command_publisher.get_logger().info("Audio saved to test_audio.wav")

        #Transcribe audio using Whisper API
        t1 = time.time()
        user_input = command_publisher.transcribe_audio_with_whisper(filename)
        t2 = time.time()
        
        command_publisher.get_logger().info(f"Time taken: {t2 - t1}")
        # If the user said 'exit', stop the loop
        if user_input and user_input.lower() == 'exit':
            print("Exiting the publisher.")

        # Publish the recognized text
        if user_input:
            command_publisher.publish_message(user_input)

        # Allow ROS2 to process the message
        rclpy.spin_once(command_publisher, timeout_sec=0.1)

        # Delay for 0.9 seconds
        time.sleep(0.9)

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    # Clean up and shutdown
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
