#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import whisper, os
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

# Whisper settings

Model = 'tiny'     # Whisper model size (tiny, base, small, medium, large)
English = True      # Use English-only model?
Translate = False   # Translate non-English to English?
SampleRate = 44100  # Stream device recording frequency
BlockSize = 30      # Block size in milliseconds
Threshold = 0.1     # Minimum volume threshold to activate listening
Vocals = [125, 1000] # Frequency range to detect sounds that could be speech
EndBlocks = 40      # Number of blocks to wait before sending to Whisper

class Transcriber(Node):
    def __init__(self, assist=None):
        super().__init__('whisper_transcriber')
        self.callback_group = ReentrantCallbackGroup()
        self.transcription_publisher = self.create_publisher(String, 'whisper_transcript', 10, callback_group=self.callback_group)

        if assist == None:  # If not being run by my assistant, just run as terminal transcriber.
            class fakeAsst(): running, talking, analyze = True, False, None
            self.asst = fakeAsst()  # anyone know a better way to do this?
        else: self.asst = assist
        self.lock = threading.Lock()
        self.running = True
        self.padding = 0
        self.prevblock = self.buffer = np.zeros((0,1))
        self.fileready = False
        print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
        self.model = whisper.load_model(f'{Model}{".en" if English else ""}')
        print("\033[90m Done.\033[0m")

    def callback(self, indata, frames, time, status):
        with self.lock:
            if status: print(status) # for debugging, prints stream errors.
            if not any(indata):
                print("\033[31mNo input or device is muted.\033[0m") #old way
                #self.running = False  # used to terminate if no input
                return
            freq = np.argmax(np.abs(np.fft.rfft(indata[:, 0]))) * SampleRate / frames
            if np.sqrt(np.mean(indata**2)) > Threshold and Vocals[0] <= freq <= Vocals[1] and not self.asst.talking:
                print('.', end='', flush=True)
                if self.padding < 1: self.buffer = self.prevblock.copy()
                self.buffer = np.concatenate((self.buffer, indata))
                self.padding = EndBlocks
            else:
                self.padding -= 1
                if self.padding > 1:
                    self.buffer = np.concatenate((self.buffer, indata))
                elif self.padding < 1 < self.buffer.shape[0] > SampleRate: # if enough silence has passed, write to file.
                    self.fileready = True
                    write('dictate.wav', SampleRate, self.buffer) # I'd rather send data to Whisper directly..
                    self.buffer = np.zeros((0,1))
                elif self.padding < 1 < self.buffer.shape[0] < SampleRate: # if recording not long enough, reset buffer.
                    self.buffer = np.zeros((0,1))
                    print("\033[2K\033[0G", end='', flush=True)
                else:
                    self.prevblock = indata.copy() #np.concatenate((self.prevblock[-int(SampleRate/10):], indata)) # SLOW

    def process(self):
        if self.fileready:
            print("\n\033[90mTranscribing..\033[0m")
            result = self.model.transcribe('dictate.wav', fp16=False, language='en' if English else '', task='translate' if Translate else 'transcribe')
            
            # Create a String message and set its data
            msg = String()
            msg.data = result['text']
            
            print(f"You said: {result['text']}")
            self.transcription_publisher.publish(msg)  # Publish the String message
            
            if self.asst.analyze is not None:
                self.asst.analyze(result['text'])
            self.fileready = False



    def listen(self):
        if not self.running:
            return

        print("\033[32mListening.. \033[37m(Ctrl+C to Quit)\033[0m")

        # Use a with statement to ensure that the InputStream is properly closed
        with sd.InputStream(
            channels=1,
            callback=self.callback,
            blocksize=int(SampleRate * BlockSize / 1000),
            samplerate=SampleRate
        ):
            # Process audio while the node is running
            while self.running and self.asst.running:
                self.process()

    def destroy_node(self):
        """Gracefully shuts down the transcriber node."""
        print("\033[93mShutting down transcriber node.\033[0m")
        self.running = False
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    transcriber = Transcriber()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(transcriber)
    
    listen_thread = threading.Thread(target=transcriber.listen)
    listen_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Only set running to False if it's still True
        if transcriber.running:
            transcriber.running = False
        listen_thread.join()
        transcriber.destroy_node()
        
        # Ensure shutdown is called only once
        if rclpy.ok():
            rclpy.shutdown()
            print("\n\033[93mQuitting..\033[0m")


if __name__ == '__main__':
    main()  
