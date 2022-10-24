#!/usr/bin/env python3

import unittest

import launch
import launch_testing.actions

from sound_play.libsoundplay import SoundClient

def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])

class TestCase(unittest.TestCase):
    def test_soundclient_constructor(self):
        s = SoundClient()
        self.assertIsNotNone(s)
