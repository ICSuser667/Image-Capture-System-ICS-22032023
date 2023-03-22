#!/usr/bin/env python3
import os
import signal
import kivy

import rospy
import subprocess
from kivy.lang import Builder
from kivymd.app import MDApp
from std_msgs.msg import String, Bool


class IcsGuiApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen = Builder.load_file('/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/src/ICS/gui.kv')

    def build(self):
        return self.screen

    # function that starts the capture of images
    def start_capture(self, *args):
        value = True
        pub.publish(value)

    def stop_capture(self, *args):
        value = False
        pub3.publish(value)

    # function to send filepaths to gui
    def set_filepaths(self, *args):
        print("filepaths for savefolders")
        filepaths = self.screen.ids.path1.text + "#" + self.screen.ids.path2.text
        pub2.publish(filepaths)


if __name__ == '__main__':
    rospy.init_node("gui_node", anonymous=True)
    pub = rospy.Publisher("start", Bool, queue_size=10)
    pub2 = rospy.Publisher("filepaths", String, queue_size=10)
    pub3 = rospy.Publisher("shutdown", Bool, queue_size=10)
    IcsGuiApp().run()
