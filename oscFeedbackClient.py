#!/usr/bin/python3
""" 
OSC Feedback Client Class

Creates an OSC Client to transmit messages to a puredata patch ToneFeedback.pd

Created By: Andrew Pohl
            Faculty of Kinesiology - Univeristy of Calgary
            December 2019
"""

from pythonosc import udp_client
from time import sleep

class OscFeedbackClient():
    def __init__(self):
        self.volume = 0
        self.notes = []
        self.client = udp_client.SimpleUDPClient('127.0.0.1', 5005)

    def setVolume(self, volume):
        self.volume = volume

    def setNotes(self, notes):
        if len(notes)>3:
            raise Exception("Please only provide 3 notes!")
        self.notes = notes

    def playTone(self):
        for i, note in enumerate(self.notes):
            midistring = "/midi" + str(i+1)
            self.client.send_message(midistring, note)

        self.client.send_message("/volume", self.volume)

    def close(self):
        self.volume = 0
        self.notes = []
        self.playTone()


        

