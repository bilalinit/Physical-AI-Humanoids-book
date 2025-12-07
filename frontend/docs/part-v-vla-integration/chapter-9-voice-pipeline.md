---
sidebar_position: 9
title: 'Chapter 9: The Ears (Voice Pipeline)'
description: 'Building voice processing pipelines for humanoid robots using OpenAI Whisper and speech recognition'
---

# Chapter 9: The Ears (Voice Pipeline)

## Introduction

Voice interaction is a crucial component for humanoid robots, enabling natural human-robot communication. This chapter explores the development of comprehensive voice processing pipelines that allow humanoid robots to hear, understand, and respond to human speech. We'll cover everything from audio capture and preprocessing to speech recognition using OpenAI Whisper and natural language processing integration.

## Audio Capture and Preprocessing

### Microphone Array Setup

For humanoid robots, proper microphone placement is crucial for effective voice capture:

```python
import pyaudio
import numpy as np
import webrtcvad
from scipy import signal

class AudioCapture:
    def __init__(self, sample_rate=16000, channels=4):
        self.sample_rate = sample_rate
        self.channels = channels
        self.audio = pyaudio.PyAudio()

        # Initialize microphone array
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=channels,
            rate=sample_rate,
            input=True,
            frames_per_buffer=1024
        )

        # Voice Activity Detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(2)  # Aggressive VAD

    def capture_audio(self):
        """Capture audio from microphone array"""
        data = self.stream.read(1024)
        audio_data = np.frombuffer(data, dtype=np.int16)

        # Reshape for multiple channels
        audio_data = audio_data.reshape(-1, self.channels)

        return audio_data

    def detect_voice_activity(self, audio_chunk):
        """Detect voice activity in audio chunk"""
        # Convert to 16kHz mono for VAD
        mono_audio = audio_chunk[:, 0]  # Use first channel

        # VAD requires 10, 20, or 30ms chunks
        chunk_size = int(self.sample_rate * 0.01)  # 10ms
        frames = [mono_audio[i:i+chunk_size] for i in range(0, len(mono_audio), chunk_size)]

        vad_results = []
        for frame in frames:
            if len(frame) == chunk_size:
                vad_result = self.vad.is_speech(frame.tobytes(), self.sample_rate)
                vad_results.append(vad_result)

        return any(vad_results)
```

### Beamforming for Directional Hearing

```python
class Beamformer:
    def __init__(self, mic_positions, sample_rate=16000):
        self.mic_positions = np.array(mic_positions)  # 3D positions of microphones
        self.sample_rate = sample_rate
        self.speed_of_sound = 343.0  # m/s

    def calculate_delays(self, source_direction, distance=1.0):
        """Calculate delays for beamforming toward source direction"""
        # Calculate distances from each mic to source
        source_pos = distance * np.array([
            np.cos(source_direction[0]) * np.cos(source_direction[1]),
            np.sin(source_direction[0]) * np.cos(source_direction[1]),
            np.sin(source_direction[1])
        ])

        distances = np.linalg.norm(self.mic_positions - source_pos, axis=1)

        # Calculate time delays
        delays = distances / self.speed_of_sound

        # Convert to samples
        delay_samples = (delays * self.sample_rate).astype(int)

        return delay_samples

    def apply_beamforming(self, multi_channel_audio, delays):
        """Apply delays and sum to form beam"""
        # Apply delays to each channel
        delayed_signals = []
        max_delay = max(delays)

        for i, delay in enumerate(delays):
            # Pad signal to align with maximum delay
            padded = np.pad(multi_channel_audio[:, i], (max_delay - delay, 0), mode='constant')
            delayed_signals.append(padded)

        # Stack and sum to form beamformed signal
        delayed_signals = np.array(delayed_signals)
        beamformed = np.sum(delayed_signals, axis=0)

        return beamformed
```

### Noise Reduction and Audio Enhancement

```python
import librosa
from scipy import signal as scipy_signal

class AudioEnhancer:
    def __init__(self):
        self.sample_rate = 16000

    def denoise_audio(self, audio_signal):
        """Apply noise reduction to audio signal"""
        # Use spectral gating for noise reduction
        stft = librosa.stft(audio_signal)
        magnitude, phase = librosa.magphase(stft)

        # Estimate noise profile from beginning of signal
        noise_profile = np.mean(magnitude[:, :50], axis=1, keepdims=True)

        # Apply spectral gating
        gain = np.maximum(0, (magnitude - noise_profile) / magnitude)
        enhanced_magnitude = magnitude * gain

        # Reconstruct signal
        enhanced_stft = enhanced_magnitude * phase
        enhanced_audio = librosa.istft(enhanced_stft)

        return enhanced_audio

    def apply_beamforming_and_enhancement(self, multi_channel_audio):
        """Combine beamforming and enhancement"""
        # First, apply beamforming to focus on speaker
        beamformer = Beamformer([
            [0, 0, 0],      # Mic 1
            [0.05, 0, 0],   # Mic 2
            [0, 0.05, 0],   # Mic 3
            [0.05, 0.05, 0] # Mic 4
        ])

        # Estimate direction of arrival (simplified)
        # In practice, use DOA estimation algorithms
        delays = beamformer.calculate_delays([0, 0])  # Front direction
        beamformed_audio = beamformer.apply_beamforming(multi_channel_audio, delays)

        # Apply noise reduction
        enhanced_audio = self.denoise_audio(beamformed_audio)

        return enhanced_audio
```

## Speech Recognition with OpenAI Whisper

### Whisper Integration

```python
import whisper
import torch
import numpy as np

class WhisperRecognizer:
    def __init__(self, model_size="medium"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Check for GPU availability
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = self.model.to(self.device)

    def transcribe_audio(self, audio_data, language="en", task="transcribe"):
        """Transcribe audio using Whisper"""
        # Convert to float32 and normalize
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32) / 32768.0

        # Transcribe using Whisper
        result = self.model.transcribe(
            audio_data,
            language=language,
            task=task,
            fp16=torch.cuda.is_available()
        )

        return result["text"], result.get("segments", [])

    def transcribe_with_timestamps(self, audio_data):
        """Transcribe with word-level timestamps"""
        # Convert to float32 and normalize
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32) / 32768.0

        # Use Whisper's detailed transcription
        result = self.model.transcribe(
            audio_data,
            word_timestamps=True,
            fp16=torch.cuda.is_available()
        )

        return result

    def continuous_transcription(self, audio_stream):
        """Process continuous audio stream"""
        transcription_buffer = []
        segment_duration = 10.0  # Process 10-second segments

        for audio_segment in audio_stream:
            # Transcribe the segment
            text, segments = self.transcribe_audio(audio_segment)

            # Add to buffer with timestamps
            for segment in segments:
                transcription_buffer.append({
                    'text': segment['text'].strip(),
                    'start': segment['start'],
                    'end': segment['end'],
                    'confidence': segment.get('avg_logprob', 0)
                })

            # Process completed sentences
            completed_sentences = self.extract_completed_sentences(transcription_buffer)

            for sentence in completed_sentences:
                yield sentence
                # Remove processed sentences from buffer
                transcription_buffer = transcription_buffer[1:]

    def extract_completed_sentences(self, buffer):
        """Extract completed sentences from buffer"""
        completed = []
        for item in buffer:
            text = item['text']
            if any(punct in text for punct in '.!?'):
                completed.append(item)
        return completed
```

### Real-time Voice Pipeline

```python
import threading
import queue
import time

class RealTimeVoicePipeline:
    def __init__(self):
        self.audio_capture = AudioCapture()
        self.audio_enhancer = AudioEnhancer()
        self.whisper_recognizer = WhisperRecognizer()

        # Processing queues
        self.raw_audio_queue = queue.Queue(maxsize=10)
        self.enhanced_audio_queue = queue.Queue(maxsize=5)
        self.transcription_queue = queue.Queue(maxsize=5)

        # Control flags
        self.running = False
        self.vad_threshold = 0.5

    def start_processing(self):
        """Start the real-time voice processing pipeline"""
        self.running = True

        # Start processing threads
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.enhancement_thread = threading.Thread(target=self._enhancement_loop)
        self.recognition_thread = threading.Thread(target=self._recognition_loop)

        self.capture_thread.start()
        self.enhancement_thread.start()
        self.recognition_thread.start()

    def _capture_loop(self):
        """Capture audio in a separate thread"""
        while self.running:
            try:
                # Capture raw audio
                raw_audio = self.audio_capture.capture_audio()

                # Check for voice activity
                if self.audio_capture.detect_voice_activity(raw_audio):
                    # Add to enhancement queue
                    if not self.raw_audio_queue.full():
                        self.raw_audio_queue.put(raw_audio, timeout=1)

                time.sleep(0.01)  # 10ms sleep

            except Exception as e:
                print(f"Capture error: {e}")
                time.sleep(0.1)

    def _enhancement_loop(self):
        """Enhance audio in a separate thread"""
        while self.running:
            try:
                # Get raw audio
                raw_audio = self.raw_audio_queue.get(timeout=1)

                # Apply enhancement
                enhanced_audio = self.audio_enhancer.apply_beamforming_and_enhancement(raw_audio)

                # Add to recognition queue
                if not self.enhanced_audio_queue.full():
                    self.enhanced_audio_queue.put(enhanced_audio, timeout=1)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Enhancement error: {e}")

    def _recognition_loop(self):
        """Perform speech recognition in a separate thread"""
        while self.running:
            try:
                # Get enhanced audio
                enhanced_audio = self.enhanced_audio_queue.get(timeout=1)

                # Transcribe audio
                text, segments = self.whisper_recognizer.transcribe_audio(enhanced_audio)

                if text.strip():  # If we have recognized text
                    transcription_result = {
                        'text': text,
                        'timestamp': time.time(),
                        'segments': segments
                    }

                    # Add to output queue
                    if not self.transcription_queue.full():
                        self.transcription_queue.put(transcription_result, timeout=1)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Recognition error: {e}")

    def get_transcription(self, timeout=1.0):
        """Get the next transcription result"""
        try:
            return self.transcription_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop_processing(self):
        """Stop the voice processing pipeline"""
        self.running = False

        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2)
        if hasattr(self, 'enhancement_thread'):
            self.enhancement_thread.join(timeout=2)
        if hasattr(self, 'recognition_thread'):
            self.recognition_thread.join(timeout=2)
```

## Voice Activity Detection and Wake Word Recognition

### Advanced VAD Implementation

```python
import webrtcvad
import collections
import numpy as np

class AdvancedVAD:
    def __init__(self, sample_rate=16000):
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(2)  # Aggressive mode
        self.sample_rate = sample_rate

        # Audio frame size (must be 10, 20, or 30ms)
        self.frame_duration = 20  # ms
        self.frame_size = int(sample_rate * self.frame_duration / 1000)

        # Voice activity tracking
        self.voice_frames = collections.deque(maxlen=100)
        self.speech_start_threshold = 3  # Minimum voiced frames to start speech
        self.speech_end_threshold = 15   # Consecutive silent frames to end speech

    def is_speech_frame(self, audio_frame):
        """Check if audio frame contains speech"""
        if len(audio_frame) != self.frame_size:
            raise ValueError(f"Frame size must be {self.frame_size}, got {len(audio_frame)}")

        # Convert to bytes for WebRTC VAD
        audio_bytes = (audio_frame.astype(np.int16)).tobytes()

        try:
            return self.vad.is_speech(audio_bytes, self.sample_rate)
        except:
            return False

    def detect_speech_segment(self, audio_stream):
        """Detect speech segments in audio stream"""
        in_speech = False
        speech_start = None
        speech_frames = []

        for i, frame in enumerate(audio_stream):
            is_speech = self.is_speech_frame(frame)
            self.voice_frames.append(is_speech)

            if not in_speech and is_speech:
                # Potential speech start
                voiced_count = sum(list(self.voice_frames)[-self.speech_start_threshold:])
                if voiced_count >= self.speech_start_threshold:
                    in_speech = True
                    speech_start = i - self.speech_start_threshold
                    speech_frames = []

            if in_speech:
                speech_frames.append(frame)

                # Check for speech end
                recent_frames = list(self.voice_frames)[-self.speech_end_threshold:]
                if len(recent_frames) >= self.speech_end_threshold:
                    silent_count = recent_frames.count(False)
                    if silent_count >= self.speech_end_threshold:
                        in_speech = False
                        yield speech_start, speech_frames
                        speech_frames = []
```

### Wake Word Detection

```python
import torch
import torch.nn as nn
import torchaudio

class WakeWordDetector(nn.Module):
    def __init__(self, num_classes=2):  # wake word vs not wake word
        super(WakeWordDetector, self).__init__()

        # CNN layers for audio feature extraction
        self.conv_layers = nn.Sequential(
            nn.Conv1d(1, 64, kernel_size=80, stride=4),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.MaxPool1d(4),

            nn.Conv1d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.MaxPool1d(4),

            nn.Conv1d(128, 256, kernel_size=3, padding=1),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.MaxPool1d(4),
        )

        # Calculate the size after convolutions
        self.fc_input_size = 256  # This needs to be calculated based on audio length

        self.fc_layers = nn.Sequential(
            nn.Linear(self.fc_input_size, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes)
        )

        self.softmax = nn.Softmax(dim=1)

    def forward(self, x):
        x = self.conv_layers(x)
        x = torch.mean(x, dim=2)  # Global average pooling
        x = self.fc_layers(x)
        return x

class WakeWordPipeline:
    def __init__(self, model_path=None):
        self.model = WakeWordDetector()
        self.transform = torchaudio.transforms.MFCC(
            sample_rate=16000,
            n_mfcc=40
        )

        if model_path:
            self.model.load_state_dict(torch.load(model_path))

        self.model.eval()

    def preprocess_audio(self, audio_data):
        """Preprocess audio for wake word detection"""
        # Convert to tensor
        if not isinstance(audio_data, torch.Tensor):
            audio_data = torch.tensor(audio_data, dtype=torch.float32)

        # Normalize
        audio_data = audio_data / torch.max(torch.abs(audio_data))

        # Reshape for model input
        if audio_data.dim() == 1:
            audio_data = audio_data.unsqueeze(0)

        return audio_data

    def detect_wake_word(self, audio_data):
        """Detect wake word in audio data"""
        processed_audio = self.preprocess_audio(audio_data)

        with torch.no_grad():
            output = self.model(processed_audio)
            probabilities = torch.softmax(output, dim=1)
            prediction = torch.argmax(probabilities, dim=1)

            confidence = probabilities[0][prediction[0]].item()

        is_wake_word = bool(prediction[0].item())

        return is_wake_word, confidence
```

## Integration with Humanoid Control

### Voice Command Processing

```python
import re
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class VoiceCommand:
    intent: str
    entities: Dict[str, str]
    confidence: float
    original_text: str

class VoiceCommandProcessor:
    def __init__(self):
        self.command_patterns = {
            'move': [
                r'move (?P<direction>\w+) (?P<distance>\d+\.?\d*) (?:meters|steps)?',
                r'go (?P<direction>\w+) (?P<distance>\d+\.?\d*) (?:meters|steps)?',
                r'walk (?P<direction>\w+) (?P<distance>\d+\.?\d*) (?:meters|steps)?'
            ],
            'turn': [
                r'turn (?P<direction>left|right) (?P<angle>\d+\.?\d*) degrees?',
                r'rotate (?P<direction>left|right) (?P<angle>\d+\.?\d*) degrees?',
                r'pivot (?P<direction>left|right)'
            ],
            'greet': [
                r'hello',
                r'hi',
                r'hey',
                r'greetings'
            ],
            'stop': [
                r'stop',
                r'freeze',
                r'hold',
                r'wait'
            ],
            'dance': [
                r'dance',
                r'twerk',
                r'boogie',
                r'move your body'
            ]
        }

    def extract_command(self, text: str) -> Optional[VoiceCommand]:
        """Extract command from text using pattern matching"""
        text_lower = text.lower().strip()

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    entities = match.groupdict()
                    return VoiceCommand(
                        intent=intent,
                        entities=entities,
                        confidence=0.9,  # High confidence for pattern matches
                        original_text=text
                    )

        # If no pattern matches, return None or use NLP for complex commands
        return self.process_complex_command(text_lower)

    def process_complex_command(self, text: str) -> Optional[VoiceCommand]:
        """Process more complex commands using semantic understanding"""
        # Simple keyword-based approach for complex commands
        if any(word in text for word in ['please', 'could you', 'can you']):
            if any(word in text for word in ['move', 'go', 'walk', 'step']):
                return VoiceCommand(
                    intent='move',
                    entities={'direction': 'forward', 'distance': '1'},
                    confidence=0.7,
                    original_text=text
                )

        return None
```

### Voice Pipeline Integration

```python
class HumanoidVoiceInterface:
    def __init__(self):
        self.voice_pipeline = RealTimeVoicePipeline()
        self.command_processor = VoiceCommandProcessor()
        self.tts_engine = None  # Text-to-speech engine

        # Wake word detection
        self.wake_word_detector = WakeWordDetector()
        self.listening_mode = False  # Whether actively listening

    def start_interaction_loop(self):
        """Start the main voice interaction loop"""
        self.voice_pipeline.start_processing()

        try:
            while True:
                # Check for wake word if not in listening mode
                if not self.listening_mode:
                    audio_chunk = self.voice_pipeline.get_transcription(timeout=0.1)
                    if audio_chunk:
                        wake_detected, confidence = self.check_wake_word(audio_chunk['text'])
                        if wake_detected and confidence > 0.8:
                            self.activate_listening()
                            self.respond("Yes, how can I help you?")

                # Process commands if in listening mode
                if self.listening_mode:
                    transcription = self.voice_pipeline.get_transcription(timeout=1.0)
                    if transcription:
                        command = self.command_processor.extract_command(transcription['text'])
                        if command:
                            self.execute_command(command)

                            # Deactivate listening after command
                            self.deactivate_listening()
                            self.respond("Command executed")

        except KeyboardInterrupt:
            print("Stopping voice interface...")
        finally:
            self.voice_pipeline.stop_processing()

    def check_wake_word(self, text):
        """Check if text contains wake word"""
        wake_words = ['robot', 'hey robot', 'assistant', 'hello robot']
        text_lower = text.lower()

        for wake_word in wake_words:
            if wake_word in text_lower:
                return True, 0.9

        return False, 0.0

    def activate_listening(self):
        """Activate listening mode"""
        self.listening_mode = True
        print("Listening activated...")

    def deactivate_listening(self):
        """Deactivate listening mode"""
        self.listening_mode = False
        print("Listening deactivated...")

    def execute_command(self, command):
        """Execute the recognized command"""
        print(f"Executing command: {command.intent} with entities: {command.entities}")

        if command.intent == 'move':
            self.move_robot(
                direction=command.entities.get('direction', 'forward'),
                distance=float(command.entities.get('distance', 1.0))
            )
        elif command.intent == 'turn':
            self.turn_robot(
                direction=command.entities.get('direction', 'left'),
                angle=float(command.entities.get('angle', 90.0))
            )
        elif command.intent == 'greet':
            self.greet()
        elif command.intent == 'stop':
            self.stop_robot()
        elif command.intent == 'dance':
            self.perform_dance()

    def respond(self, text):
        """Generate voice response"""
        print(f"Robot says: {text}")
        # In a real implementation, this would use TTS
        # self.tts_engine.speak(text)

    def move_robot(self, direction, distance):
        """Move the robot in specified direction"""
        print(f"Moving {direction} for {distance} meters")
        # Implementation would send commands to robot's navigation system

    def turn_robot(self, direction, angle):
        """Turn the robot"""
        print(f"Turning {direction} by {angle} degrees")
        # Implementation would send rotation commands

    def greet(self):
        """Perform greeting action"""
        print("Greeting gesture")
        # Implementation would perform greeting animation

    def stop_robot(self):
        """Stop robot movement"""
        print("Stopping robot")
        # Implementation would send stop command

    def perform_dance(self):
        """Perform dance movement"""
        print("Performing dance")
        # Implementation would execute dance routine
```

## Performance Optimization

### Efficient Processing Pipeline

```python
class OptimizedVoicePipeline:
    def __init__(self):
        # Use smaller models for faster processing
        self.whisper_model = whisper.load_model("base")  # Smaller model
        self.sample_rate = 16000

        # Processing buffers
        self.audio_buffer = np.zeros(16000 * 5)  # 5 seconds buffer
        self.buffer_index = 0

        # Processing flags
        self.processing_lock = threading.Lock()
        self.last_process_time = 0
        self.process_interval = 2.0  # Process every 2 seconds

    def add_audio_chunk(self, audio_chunk):
        """Add audio chunk to processing buffer"""
        with self.processing_lock:
            chunk_size = len(audio_chunk)

            # If buffer would overflow, shift and add
            if self.buffer_index + chunk_size > len(self.audio_buffer):
                # Shift buffer
                remaining = len(self.audio_buffer) - self.buffer_index
                if remaining > 0:
                    self.audio_buffer[:remaining] = self.audio_buffer[self.buffer_index:self.buffer_index + remaining]
                self.buffer_index = remaining
            else:
                # Add to buffer
                self.audio_buffer[self.buffer_index:self.buffer_index + chunk_size] = audio_chunk
                self.buffer_index += chunk_size

    def should_process_audio(self):
        """Check if enough time has passed to process audio"""
        current_time = time.time()
        return (current_time - self.last_process_time) >= self.process_interval

    def process_audio_buffer(self):
        """Process the current audio buffer"""
        if self.buffer_index < self.sample_rate:  # At least 1 second of audio
            return None

        with self.processing_lock:
            # Get audio segment to process
            audio_segment = self.audio_buffer[:self.buffer_index].copy()
            self.buffer_index = 0  # Reset buffer

        # Transcribe the audio
        if len(audio_segment) > 0:
            text, segments = self.transcribe_audio_segment(audio_segment)
            self.last_process_time = time.time()
            return text, segments

        return None

    def transcribe_audio_segment(self, audio_segment):
        """Transcribe audio segment using Whisper"""
        # Normalize audio
        if audio_segment.dtype != np.float32:
            audio_segment = audio_segment.astype(np.float32) / 32768.0

        # Transcribe
        result = self.whisper_model.transcribe(
            audio_segment,
            language="en",
            task="transcribe",
            fp16=False  # Use float32 for CPU
        )

        return result["text"], result.get("segments", [])
```

## Voice Pipeline Code Examples

### 1. Complete Voice Processing Pipeline

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import pyaudio
import numpy as np
import whisper
import openai
import threading
import queue
import time
from collections import deque
import webrtcvad
from scipy import signal
import torch

class VoicePipelineNode(Node):
    def __init__(self):
        super().__init__('voice_pipeline_node')

        # Initialize audio parameters
        self.sample_rate = 16000
        self.channels = 1
        self.chunk_size = 1024
        self.audio_format = pyaudio.paInt16

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio input stream
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("base.en")  # Use "base.en" for English only

        # Voice Activity Detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(2)  # Aggressive mode

        # Audio buffer for processing
        self.audio_buffer = deque(maxlen=32000)  # 2 seconds at 16kHz
        self.processing_queue = queue.Queue(maxsize=10)

        # ROS publishers
        self.speech_text_pub = self.create_publisher(String, '/robot/speech_text', 10)
        self.command_pub = self.create_publisher(String, '/robot/command', 10)

        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Voice pipeline node initialized')

    def capture_audio(self):
        """Continuously capture audio from microphone"""
        while rclpy.ok():
            try:
                # Read audio data
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.int16)

                # Add to buffer
                for sample in audio_data:
                    self.audio_buffer.append(sample)

                # Check for voice activity
                if self.is_voice_active(audio_data):
                    # Send to processing queue
                    try:
                        self.processing_queue.put(audio_data, block=False)
                    except queue.Full:
                        pass  # Skip if queue is full

            except Exception as e:
                self.get_logger().error(f'Error in audio capture: {str(e)}')
                time.sleep(0.01)

    def is_voice_active(self, audio_data):
        """Check if voice is active using VAD"""
        # Convert to bytes for VAD
        audio_bytes = audio_data.tobytes()

        # VAD requires 10, 20, or 30 ms frames
        frame_size = int(self.sample_rate * 0.01)  # 10ms frame
        if len(audio_bytes) >= frame_size:
            frames = [audio_bytes[i:i+frame_size] for i in range(0, len(audio_bytes), frame_size)]
            for frame in frames:
                if len(frame) == frame_size:
                    if self.vad.is_speech(frame, self.sample_rate):
                        return True
        return False

    def process_audio(self):
        """Process audio data for speech recognition"""
        while rclpy.ok():
            try:
                # Get audio data from queue
                audio_data = self.processing_queue.get(timeout=1.0)

                # Convert to float32
                audio_float = audio_data.astype(np.float32) / 32768.0

                # Transcribe using Whisper
                result = self.whisper_model.transcribe(
                    audio_float,
                    language="en",
                    task="transcribe",
                    fp16=False
                )

                # Publish recognized text
                if result and result["text"].strip():
                    text_msg = String()
                    text_msg.data = result["text"].strip()
                    self.speech_text_pub.publish(text_msg)

                    # Process for commands
                    self.process_command(result["text"])

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {str(e)}')

    def process_command(self, text):
        """Process recognized text for commands"""
        # Simple command recognition (in practice, use more sophisticated NLP)
        text_lower = text.lower()

        if "move forward" in text_lower:
            cmd_msg = String()
            cmd_msg.data = "move_forward"
            self.command_pub.publish(cmd_msg)
        elif "turn left" in text_lower:
            cmd_msg = String()
            cmd_msg.data = "turn_left"
            self.command_pub.publish(cmd_msg)
        elif "turn right" in text_lower:
            cmd_msg = String()
            cmd_msg.data = "turn_right"
            self.command_pub.publish(cmd_msg)
        elif "stop" in text_lower:
            cmd_msg = String()
            cmd_msg.data = "stop"
            self.command_pub.publish(cmd_msg)

    def destroy_node(self):
        """Clean up resources"""
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    voice_pipeline = VoicePipelineNode()

    try:
        rclpy.spin(voice_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        voice_pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Advanced Voice Command Processing

```python
import openai
from transformers import pipeline, AutoTokenizer, AutoModelForSequenceClassification
import json
import re

class VoiceCommandProcessor:
    def __init__(self):
        # Initialize NLP models
        self.tokenizer = AutoTokenizer.from_pretrained("microsoft/DialoGPT-medium")
        self.nlp_model = AutoModelForSequenceClassification.from_pretrained(
            "microsoft/DialoGPT-medium"
        )

        # Initialize intent classification pipeline
        self.intent_classifier = pipeline(
            "text-classification",
            model="microsoft/DialoGPT-medium"
        )

        # Define command mappings
        self.command_mappings = {
            "move": ["move forward", "go forward", "walk forward", "step forward"],
            "turn": ["turn left", "turn right", "rotate left", "rotate right", "pivot left", "pivot right"],
            "stop": ["stop", "halt", "freeze", "pause"],
            "speak": ["speak", "talk", "say", "voice"],
            "look": ["look", "see", "find", "search", "detect"],
            "greet": ["hello", "hi", "greet", "wave", "hello robot"]
        }

    def process_voice_command(self, text):
        """Process voice command and extract intent and parameters"""
        # Clean the text
        cleaned_text = self.clean_text(text)

        # Classify intent
        intent = self.classify_intent(cleaned_text)

        # Extract parameters
        params = self.extract_parameters(cleaned_text, intent)

        return {
            "intent": intent,
            "parameters": params,
            "original_text": text,
            "cleaned_text": cleaned_text
        }

    def clean_text(self, text):
        """Clean and normalize text input"""
        # Convert to lowercase
        text = text.lower()

        # Remove extra whitespace
        text = ' '.join(text.split())

        # Remove common filler words
        fillers = ["um", "uh", "like", "you know", "so", "well", "actually"]
        for filler in fillers:
            text = text.replace(filler, "")

        # Remove punctuation (except for specific cases)
        text = re.sub(r'[^\w\s]', ' ', text)

        # Remove extra spaces again after punctuation removal
        text = ' '.join(text.split())

        return text.strip()

    def classify_intent(self, text):
        """Classify the intent of the voice command"""
        # Check against predefined command mappings
        for intent, phrases in self.command_mappings.items():
            for phrase in phrases:
                if phrase in text:
                    return intent

        # If no exact match, use more sophisticated NLP
        # For this example, return a default intent
        return "unknown"

    def extract_parameters(self, text, intent):
        """Extract parameters from the command"""
        params = {}

        if intent == "move":
            # Extract distance if specified
            distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(meters?|m|steps?)', text)
            if distance_match:
                params['distance'] = float(distance_match.group(1))
                params['unit'] = distance_match.group(2)
            else:
                params['distance'] = 1.0  # Default distance
                params['unit'] = 'm'

        elif intent == "turn":
            # Extract direction
            if "left" in text:
                params['direction'] = "left"
            elif "right" in text:
                params['direction'] = "right"
            else:
                params['direction'] = "unknown"

            # Extract angle if specified
            angle_match = re.search(r'(\d+)\s*(degrees?|deg)', text)
            if angle_match:
                params['angle'] = int(angle_match.group(1))
            else:
                params['angle'] = 90  # Default turn angle

        elif intent == "greet":
            # Extract greeting type
            if "wave" in text:
                params['type'] = "wave"
            elif "nod" in text:
                params['type'] = "nod"
            else:
                params['type'] = "verbal"

        return params

    def generate_robot_response(self, intent, params):
        """Generate a verbal response for the robot to speak"""
        responses = {
            "move": f"Moving forward {params.get('distance', 1)} meters.",
            "turn": f"Turning {params.get('direction', 'unknown')} by {params.get('angle', 90)} degrees.",
            "stop": "Stopping movement.",
            "greet": "Hello! How can I assist you today?",
            "unknown": f"I didn't understand the command: {intent}. Could you please repeat?"
        }

        return responses.get(intent, responses["unknown"])
```

### 3. Voice Pipeline Integration with ROS 2 Services

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RclDuration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from audio_common_msgs.action import AudioStream
import threading
import time

class VoicePipelineService(Node):
    def __init__(self):
        super().__init__('voice_pipeline_service')

        # Initialize voice pipeline components
        self.voice_pipeline = VoicePipelineNode()
        self.command_processor = VoiceCommandProcessor()

        # Service to start/stop voice recognition
        self.start_service = self.create_service(
            String,
            'start_voice_recognition',
            self.start_voice_recognition_callback
        )
        self.stop_service = self.create_service(
            String,
            'stop_voice_recognition',
            self.stop_voice_recognition_callback
        )

        # Action server for voice commands
        self._action_server = ActionServer(
            self,
            AudioStream,
            'voice_command',
            execute_callback=self.execute_voice_command,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for voice responses
        self.response_publisher = self.create_publisher(String, '/robot/voice_response', 10)

        self.is_listening = False
        self.listening_thread = None

        self.get_logger().info('Voice pipeline service initialized')

    def start_voice_recognition_callback(self, request, response):
        """Start voice recognition service"""
        if not self.is_listening:
            self.is_listening = True
            self.listening_thread = threading.Thread(target=self.listen_for_commands)
            self.listening_thread.daemon = True
            self.listening_thread.start()
            response.data = "Voice recognition started"
        else:
            response.data = "Voice recognition already active"
        return response

    def stop_voice_recognition_callback(self, request, response):
        """Stop voice recognition service"""
        if self.is_listening:
            self.is_listening = False
            if self.listening_thread:
                self.listening_thread.join(timeout=2.0)  # Wait up to 2 seconds
            response.data = "Voice recognition stopped"
        else:
            response.data = "Voice recognition was not active"
        return response

    def listen_for_commands(self):
        """Listen for voice commands in a separate thread"""
        while self.is_listening and rclpy.ok():
            # In a real implementation, this would interface with the audio capture
            # For this example, we'll just sleep
            time.sleep(0.1)

    def goal_callback(self, goal_request):
        """Accept or reject voice command goals"""
        self.get_logger().info('Received voice command goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject voice command cancel requests"""
        self.get_logger().info('Received cancel request for voice command')
        return CancelResponse.ACCEPT

    async def execute_voice_command(self, goal_handle):
        """Execute a voice command goal"""
        self.get_logger().info('Executing voice command goal')

        feedback_msg = AudioStream.Feedback()
        result = AudioStream.Result()

        # Process the voice command
        command_data = goal_handle.request.command
        processed_command = self.command_processor.process_voice_command(command_data)

        # Generate robot response
        response_text = self.command_processor.generate_robot_response(
            processed_command['intent'],
            processed_command['parameters']
        )

        # Publish the response
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)

        # Set feedback
        feedback_msg.status = f"Processed command: {processed_command['intent']}"
        goal_handle.publish_feedback(feedback_msg)

        # Simulate command execution
        for i in range(5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Voice command canceled')
                return result

            # Publish feedback
            feedback_msg.status = f"Executing command step {i+1}/5"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # Complete the goal
        goal_handle.succeed()
        result.success = True
        result.message = f"Command '{processed_command['intent']}' executed successfully"

        self.get_logger().info(f'Voice command completed: {result.message}')
        return result

    def destroy_node(self):
        """Clean up resources"""
        if self._action_server:
            self._action_server.destroy()
        super().destroy_node()
```

### 4. Voice Pipeline Configuration

```yaml
# Voice pipeline configuration for humanoid robot
voice_pipeline:
  ros__parameters:
    # Audio parameters
    sample_rate: 16000
    channels: 1
    chunk_size: 1024
    audio_format: "int16"

    # VAD (Voice Activity Detection) parameters
    vad_mode: 2  # 0-3, 3 is most aggressive
    vad_frame_duration: 10  # ms (10, 20, or 30)

    # Whisper model parameters
    whisper_model_size: "base.en"  # tiny, base, small, medium, large
    whisper_language: "en"
    whisper_task: "transcribe"

    # Processing parameters
    buffer_size: 32000  # 2 seconds at 16kHz
    max_queue_size: 10
    processing_interval: 0.1  # seconds

    # Command processing
    command_timeout: 5.0  # seconds to wait for command completion
    min_confidence: 0.7   # minimum confidence for command recognition

    # Audio enhancement
    noise_suppression: 2  # 0-4, higher is more aggressive
    auto_gain_control: true
    echo_cancellation: true
```

## Summary

The voice pipeline for humanoid robots involves multiple sophisticated components working together: audio capture and enhancement, voice activity detection, speech recognition using OpenAI Whisper, and natural language processing for command understanding. By implementing beamforming, noise reduction, and efficient processing pipelines, humanoid robots can effectively hear and understand human speech in real-world environments. The integration with robot control systems enables natural human-robot interaction, making humanoid robots more accessible and useful in various applications.