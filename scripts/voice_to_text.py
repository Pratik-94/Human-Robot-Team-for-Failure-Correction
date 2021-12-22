#! /usr/bin/env python3
import speech_recognition as sr
import pyttsx3
import time


class VoicetoText(object):

    def __init__(self):
        self._r = sr.Recognizer()
        self.text = None

    def convert_voice_to_text(self):
        with sr.Microphone() as voice:

            audio_data = self._r.record(voice,duration=5)
            text = self._r.recognize_google(audio_data)
            print(text)


class TexttoVoice(object):

    def __init__(self):

        self.engine = pyttsx3.init('espeak')

        self.voices = self.engine.getProperty('voices')
        #print(self.voices)

        #Set Robot's Voice Params
        self.engine.setProperty('rate',125)
        self.engine.setProperty('volume',1.0)
        #self.engine.setProperty('voice',self.voices[0].id)

    def say_something(self):
        self.engine.say("Hello sir, how may I help you?")
        self.engine.runAndWait()

    def __del__(self):
        pass


def main():

    obj = TexttoVoice()

    obj.say_something()

    time.sleep(2)

    

if __name__ == "__main__":
    main()