---
sidebar_position: 21
title: "Voice-to-Action using OpenAI Whisper for voice commands"
---

# Voice-to-Action using OpenAI Whisper for voice commands

## Overview

Voice-to-Action systems enable robots to receive and execute spoken commands from users. OpenAI Whisper, a state-of-the-art speech recognition system, serves as a key component in transforming spoken language into text that can be processed by natural language understanding systems. This capability is fundamental to creating intuitive human-robot interaction.

## Introduction to OpenAI Whisper

### Key Features
- **Multilingual Support**: Understands multiple languages and accents
- **Robust Recognition**: Performs well in noisy environments
- **Real-time Processing**: Low-latency speech-to-text conversion
- **Context Awareness**: Better understanding of conversation context
- **Open Source**: Available for research and commercial use

### Whisper Architecture
```
Audio Input → Feature Extraction → Encoder → Decoder → Text Output
```

### Model Variants
- **tiny**: Fastest, lowest accuracy (39M parameters)
- **base**: Good balance of speed and accuracy (74M parameters)
- **small**: Better accuracy (244M parameters)
- **medium**: High accuracy (769M parameters)
- **large**: Highest accuracy (1550M parameters)

## Integration with Robotic Systems

### Basic Whisper Integration
```python
import whisper
import torch

# Load the Whisper model
model = whisper.load_model("base")

def transcribe_audio(audio_file_path):
    # Load audio and pad/trim it to fit 30 seconds
    audio = whisper.load_audio(audio_file_path)
    audio = whisper.pad_or_trim(audio)

    # Make log-Mel spectrogram and move to the same device as the model
    mel = whisper.log_mel_spectrogram(audio).to(model.device)

    # Decode the audio
    options = whisper.DecodingOptions()
    result = whisper.decode(model, mel, options)

    return result.text
```

### Real-time Voice Processing
```python
import pyaudio
import wave
import threading
import queue

class VoiceCommandProcessor:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.audio_queue = queue.Queue()
        self.is_listening = False

    def start_listening(self):
        self.is_listening = True
        # Start audio recording thread
        recording_thread = threading.Thread(target=self.record_audio)
        recording_thread.start()

    def record_audio(self):
        # Initialize PyAudio
        p = pyaudio.PyAudio()

        # Open stream
        stream = p.open(format=pyaudio.paInt16,
                       channels=1,
                       rate=16000,
                       input=True,
                       frames_per_buffer=1024)

        while self.is_listening:
            data = stream.read(1024)
            self.audio_queue.put(data)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def process_voice_command(self):
        # Process accumulated audio data
        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())

        if audio_data:
            # Convert to numpy array and process
            import numpy as np
            audio_array = np.frombuffer(b''.join(audio_data), dtype=np.int16)
            audio_float = audio_array.astype(np.float32) / 32768.0

            # Transcribe using Whisper
            result = self.model.transcribe_from_audio(audio_float)
            return result["text"]
        return None
```

## Voice Command Processing Pipeline

### Complete Processing Flow
```
1. Voice Activity Detection
   ↓
2. Audio Preprocessing
   ↓
3. Speech Recognition (Whisper)
   ↓
4. Natural Language Processing
   ↓
5. Command Interpretation
   ↓
6. Action Planning
   ↓
7. Robot Execution
```

### Voice Activity Detection
```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, aggressiveness=2):
        self.vad = webrtcvad.Vad(aggressiveness)
        self.ring_buffer = collections.deque(maxlen=30)
        self.triggered = False
        self.vad_frames = []

    def is_speech(self, audio_frame, sample_rate=16000):
        # Check if the frame contains speech
        return self.vad.is_speech(audio_frame, sample_rate)

    def detect_voice_activity(self, audio_data):
        # Process audio in chunks and detect voice activity
        chunk_size = 160  # 10ms at 16kHz
        for i in range(0, len(audio_data), chunk_size):
            chunk = audio_data[i:i+chunk_size]
            if len(chunk) < chunk_size:
                continue

            is_speech = self.is_speech(chunk)
            self.ring_buffer.append((chunk, is_speech))

            if not self.triggered:
                if is_speech:
                    self.triggered = True
                    self.vad_frames = []
                    # Collect frames from before voice activity
                    for prev_chunk, _ in self.ring_buffer:
                        self.vad_frames.append(prev_chunk)
            elif self.triggered and not is_speech:
                # End of speech detected
                if len(self.vad_frames) > 10:  # Minimum speech duration
                    return b''.join(self.vad_frames)
                else:
                    self.triggered = False
                    self.vad_frames = []

        return None
```

## Noise Reduction and Audio Preprocessing

### Audio Enhancement
```python
import numpy as np
from scipy import signal

class AudioPreprocessor:
    def __init__(self):
        # Design a bandpass filter for human voice frequencies (300-3400 Hz)
        self.nyquist = 8000  # Half of 16kHz sample rate
        self.low_freq = 300 / self.nyquist
        self.high_freq = 3400 / self.nyquist
        self.b, self.a = signal.butter(4, [self.low_freq, self.high_freq],
                                      btype='band', analog=False)

    def preprocess_audio(self, audio_data):
        # Apply bandpass filter to focus on voice frequencies
        filtered_audio = signal.filtfilt(self.b, self.a, audio_data)

        # Normalize audio levels
        max_val = np.max(np.abs(filtered_audio))
        if max_val > 0:
            normalized_audio = filtered_audio / max_val
        else:
            normalized_audio = filtered_audio

        return normalized_audio

    def remove_silence(self, audio_data, threshold=0.01):
        # Remove silence from the beginning and end of audio
        energy = np.abs(audio_data)
        non_silent = energy > threshold

        # Find first and last non-silent samples
        indices = np.where(non_silent)[0]
        if len(indices) > 0:
            start = indices[0]
            end = indices[-1] + 1
            return audio_data[start:end]
        else:
            return audio_data
```

## Command Validation and Error Handling

### Command Validation Pipeline
```python
class CommandValidator:
    def __init__(self):
        self.valid_commands = [
            "move to", "go to", "navigate to",
            "pick up", "grasp", "take",
            "put down", "place", "drop",
            "turn left", "turn right", "rotate",
            "stop", "wait", "pause"
        ]

    def validate_command(self, transcribed_text):
        # Check if the command is likely valid
        text_lower = transcribed_text.lower()

        # Check for command keywords
        has_valid_command = any(keyword in text_lower for keyword in self.valid_commands)

        # Check for minimum confidence indicators
        confidence_indicators = [
            "please", "robot", "now", "immediately"
        ]

        has_indicators = any(indicator in text_lower for indicator in confidence_indicators)

        return has_valid_command or has_indicators, transcribed_text

    def handle_recognition_errors(self, audio_data, max_attempts=3):
        # Try multiple Whisper model sizes for better accuracy
        models = ["base", "small", "medium"]

        for i, model_size in enumerate(models[:max_attempts]):
            try:
                model = whisper.load_model(model_size)
                result = model.transcribe(audio_data)
                text = result["text"].strip()

                if text and len(text) > 3:  # Reasonable command length
                    is_valid, validated_text = self.validate_command(text)
                    if is_valid:
                        return validated_text

            except Exception as e:
                print(f"Whisper model {model_size} failed: {e}")
                continue

        return None  # Unable to get valid command
```

## Integration with ROS 2

### Voice Command Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers and subscribers
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)
        self.audio_subscriber = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10)

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Voice activity detection
        self.vad = VoiceActivityDetector()
        self.preprocessor = AudioPreprocessor()
        self.validator = CommandValidator()

        # Command processing thread
        self.command_thread = threading.Thread(target=self.process_commands)
        self.command_thread.daemon = True
        self.command_thread.start()

    def audio_callback(self, msg):
        # Process incoming audio data
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        processed_audio = self.preprocessor.preprocess_audio(audio_data)

        # Check for voice activity
        speech_data = self.vad.detect_voice_activity(processed_audio.tobytes())
        if speech_data is not None:
            # Transcribe the speech
            transcribed = self.model.transcribe(speech_data)
            command_text = transcribed["text"]

            # Validate and publish command
            is_valid, validated_command = self.validator.validate_command(command_text)
            if is_valid:
                cmd_msg = String()
                cmd_msg.data = validated_command
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info(f'Published command: {validated_command}')

    def process_commands(self):
        # Additional command processing in separate thread
        while rclpy.ok():
            # Process any queued commands
            time.sleep(0.1)  # Prevent busy waiting
```

## Performance Optimization

### Real-time Processing Considerations
- **Model Selection**: Balance accuracy vs. processing speed
- **Audio Buffering**: Optimize buffer sizes for real-time performance
- **GPU Acceleration**: Use CUDA for faster Whisper inference
- **Model Quantization**: Reduce model size for embedded systems

### GPU Acceleration Example
```python
# Enable GPU acceleration for Whisper
device = "cuda" if torch.cuda.is_available() else "cpu"
model = whisper.load_model("base").to(device)

# Process audio with GPU acceleration
with torch.no_grad():
    result = model.transcribe(audio_file, device=device)
```

## Privacy and Security Considerations

### Data Protection
- **Local Processing**: Process audio on-device when possible
- **Encryption**: Encrypt audio data in transit
- **Minimal Data**: Only process necessary audio segments
- **User Consent**: Obtain consent for voice data processing

### Security Measures
- **Access Control**: Restrict who can issue voice commands
- **Command Validation**: Verify commands before execution
- **Safety Checks**: Implement safety constraints on actions
- **Audit Logging**: Track voice command execution

## Testing and Validation

### Performance Testing
- **Accuracy Testing**: Measure speech recognition accuracy
- **Latency Testing**: Measure command processing time
- **Robustness Testing**: Test in various acoustic conditions
- **Edge Case Testing**: Test with unusual commands

### Quality Assurance
- **Command Coverage**: Ensure all supported commands work
- **Error Handling**: Verify proper error handling
- **User Experience**: Test with real users
- **Safety Validation**: Confirm safe robot behavior

## Best Practices

### Implementation Best Practices
- **Modular Design**: Separate audio processing, recognition, and command execution
- **Error Resilience**: Handle recognition failures gracefully
- **User Feedback**: Provide clear feedback for recognized commands
- **Configurable Sensitivity**: Allow adjustment of voice detection parameters

### System Design Best Practices
- **Real-time Processing**: Optimize for low-latency response
- **Resource Management**: Efficiently manage computational resources
- **Scalability**: Design for multiple robots or users
- **Maintainability**: Write clean, well-documented code

## Summary

Voice-to-Action systems using OpenAI Whisper enable natural human-robot interaction through spoken commands. By properly implementing audio preprocessing, voice activity detection, and command validation, robots can reliably understand and execute user commands. The integration with ROS 2 allows these voice commands to be seamlessly incorporated into robotic control systems, enabling intuitive and accessible robot operation.