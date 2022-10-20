#!/usr/bin/env python3

# **********************************************************
#  Software License Agreement (BSD License)
# 
#   Copyright (c) 2009, Willow Garage, Inc.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# **********************************************************

import os
import sys
import yaml
import threading
import time
import traceback

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names

import rclpy.action
import rclpy.duration
import rclpy.logging
import rclpy.node

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sound_play.action import SoundRequest as SoundRequestAction
from sound_play.msg import SoundRequest
from sound_play.sound_type import SoundType

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import GObject as GObject
    from gi.repository import Gst as Gst
except Exception:
    str = """
**************************************************************
Error opening pygst. Is gstreamer installed?
**************************************************************
"""
    rclpy.logging.get_logger('sound_play').fatal(str)
    exit(1)


class SoundPlayNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('sound_play')
        Gst.init(None)

        # Start gobject thread to receive gstreamer messages
        GObject.threads_init()
        self.g_loop = threading.Thread(target=GObject.MainLoop().run)
        self.g_loop.daemon = True
        self.g_loop.start()

        self.declare_parameter('loop_rate', 100)
        self.declare_parameter('device', 'default')
        self.declare_parameter('default_voice', None)
        self.declare_parameter('plugin', 'sound_play/festival_plugin')
        self.loop_rate = self.get_parameter('loop_rate').value
        self.device = self.get_parameter('device').value
        self.loop_rate = self.get_parameter('default_voice').value
        self.device = self.get_parameter('plugin').value

        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1)
        rootdir = os.path.join(
            get_package_share_directory('sound_play'), 'sounds')

        # load plugin
        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
            if os.path.isfile(package_file_path):
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    if export.tagname == 'sound_play':
                        if 'plugin' in export.attributes:
                            plugin_path = export.attributes['plugin']
                            plugin_yamls += plugin_path
                            for plugin_y in plugin_yaml:
                                self.get_logger.debug("Loading plugin in {}".format(plugin_y))

        plugin_dict = {}
        for plugin_yaml in plugin_yamls:
            if not os.path.exists(plugin_yaml):
                self.get_logger.error(
                    'Failed to load plugin yaml: {}'.format(plugin_yaml))
                self.get_logger.error(
                    'Missing plugin yaml: {}'.format(plugin_yaml))
                continue
            with open(plugin_yaml) as f:
                plugin_descs = yaml.safe_load(f)
            for plugin_desc in plugin_descs:
                plugin_dict[plugin_desc['name']] = plugin_desc['module']

        self.plugin = None
        if self.plugin_name in plugin_dict.keys():
            plugin_module = plugin_dict[self.plugin_name]
            mod = __import__(plugin_module.split('.')[0])
            for sub_mod in plugin_module.split('.')[1:]:
                mod = getattr(mod, sub_mod)
            self.plugin = mod()

        self.builtinsoundparams = {
            SoundRequest.BACKINGUP: (
                os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
            SoundRequest.NEEDS_UNPLUGGING: (
                os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING: (
                os.path.join(rootdir, 'NEEDS_PLUGGING.ogg'), 1),
            SoundRequest.NEEDS_UNPLUGGING_BADLY: (
                os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
            SoundRequest.NEEDS_PLUGGING_BADLY: (
                os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
        }

        self.no_error = True
        self.initialized = False
        self.active_sounds = 0

        self.mutex = threading.Lock()
        self.sub = self.create_subscription(
            SoundRequest, "robotsound", self.callback, 10)
        self._as = rclpy.action.ActionServer(
            self, SoundRequestAction, 'sound_play',
            execute_callback=self.execute_cb,
            handle_accepted_callback=self.handle_accepted_cb)

        # For ros startup race condition
        self.sleep(0.5)
        self.diagnostics(1)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._execute_lock = threading.Lock()
        self.timer = self.create_timer(0.1, self.spin_once)

    def spin_once(self):
        self.init_vars()
        self.no_error = True
        self.initialized = True
        with self.mutex:
            try:
                self.idle_loop()
            except Exception as e:
                self.get_logger().error(
                    'Exception in idle_loop: %s' % str(e))
        self.diagnostics(2)

    def stopdict(self, dict):
        for sound in dict.values():
            sound.stop()

    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def select_sound(self, data):
        if data.sound == SoundRequest.PLAY_FILE:
            if not data.arg2:
                if data.arg not in self.filesounds.keys():
                    self.get_logger().debug(
                        'command for uncached wave: "%s"' % data.arg)
                    try:
                        self.filesounds[data.arg] = SoundType(
                            self, data.arg, self.device, data.volume)
                    except Exception:
                        self.get_logger().error(
                            'Error setting up to play "%s".'
                            'Does this file exist on the machine'
                            'on which sound_play is running?' % data.arg)
                        return
                else:
                    self.get_logger().debug('command for cached wave: "%s"' % data.arg)
                    filesound = self.filesounds[data.arg]
                    if filesound.sound.get_property('volume') != data.volume:
                        self.get_logger().debug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        filesound.sound.set_property('volume', data.volume)
                sound = self.filesounds[data.arg]
            else:
                absfilename = os.path.join(
                    get_package_share_directory(data.arg2), data.arg)
                if absfilename not in self.filesounds.keys():
                    self.get_logger().debug(
                        'command for uncached wave: "%s"' % absfilename)
                    try:
                        self.filesounds[absfilename] = SoundType(
                            self, absfilename, self.device, data.volume)
                    except Exception:
                        self.get_logger().error(
                            'Error setting up to play "%s" from package "%s".'
                            'Does this file exist on the machine '
                            'on which sound_play is running?'
                            % (data.arg, data.arg2))
                        return
                else:
                    self.get_logger().debug(
                        'command for cached wave: "%s"' % absfilename)
                    filesound = self.filesounds[absfilename]
                    if filesound.sound.get_property('volume') != data.volume:
                        self.get_logger().debug(
                            'volume for cached wave has changed,'
                            'resetting volume')
                        filesound.sound.set_property('volume', data.volume)
                sound = self.filesounds[absfilename]

        elif data.sound == SoundRequest.SAY:
            voice_key = data.arg + '---' + data.arg2
            if voice_key not in self.voicesounds.keys():
                self.get_logger().debug('command for uncached text: "%s"' % voice_key)
                if self.plugin is None:
                    self.get_logger().error(
                        'Plugin is not found {}.'.format(self.plugin_name))
                else:
                    if data.arg2 == '':
                        voice = self.default_voice
                    else:
                        voice = data.arg2
                    wavfilename = self.plugin.sound_play_say_plugin(
                        data.arg, voice)
                    if wavfilename is None:
                        self.get_logger().error('Failed to generate wavfile.')
                    else:
                        self.voicesounds[voice_key] = SoundType(
                            self, wavfilename, self.device, data.volume)
            else:
                self.get_logger().debug('command for cached text: "%s"' % voice_key)
                voicesound = self.voicesounds[voice_key]
                if voicesound.sound.get_property('volume') != data.volume:
                    self.get_logger().debug(
                        'volume for cached text has changed, resetting volume')
                    voicesound.sound.set_property('volume', data.volume)
            sound = self.voicesounds[voice_key]

        else:
            self.get_logger().debug('command for builtin wave: %i' % data.sound)
            if ((data.sound in self.builtinsounds and
                 data.volume != self.builtinsounds[data.sound].volume)
                    or data.sound not in self.builtinsounds):
                params = self.builtinsoundparams[data.sound]
                volume = data.volume
                # use the second param as a scaling for the input volume
                if params[1] != 1:
                    volume = (volume + params[1])/2
                self.builtinsounds[data.sound] = SoundType(
                    self, params[0], self.device, volume)
            sound = self.builtinsounds[data.sound]
        if sound.staleness != 0 and data.command != SoundRequest.PLAY_STOP:
            # This sound isn't counted in active_sounds
            self.get_logger().debug("activating %i %s" % (data.sound, data.arg))
            self.active_sounds = self.active_sounds + 1
            sound.staleness = 0
        return sound

    def callback(self, data):
        if not self.initialized:
            return
        self.mutex.acquire()
        try:
            if (data.sound == SoundRequest.ALL
                    and data.command == SoundRequest.PLAY_STOP):
                self.stopall()
            else:
                sound = self.select_sound(data)
                sound.command(data.command)
        except Exception as e:
            self.get_logger().error('Exception in callback: %s' % str(e))
            self.get_logger().info(traceback.format_exc())
        finally:
            self.mutex.release()
            self.get_logger().debug("done callback")

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for key, sound in iter(dict.items()):
            try:
                staleness = sound.get_staleness()
            except Exception as e:
                self.get_logger().error(
                    'Exception in cleanupdict for sound (%s): %s'
                    % (str(key), str(e)))
                # Something is wrong. Let's purge and try again.
                staleness = 100
            # print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
            # Sound is playing
            if staleness == 0:
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
            self.get_logger().debug('Purging %s from cache' % key)
            # clean up resources
            dict[key].dispose()
            del dict[key]

    def cleanup(self):
        with self.mutex:
            try:
                self.active_sounds = 0
                self.cleanupdict(self.filesounds)
                self.cleanupdict(self.voicesounds)
                self.cleanupdict(self.builtinsounds)
            except Exception:
                self.get_logger().info(
                    'Exception in cleanup: %s' % sys.exc_info()[0])

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = self.get_name() + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing" % self.active_sounds
                ds.values.append(
                    KeyValue(
                        key="Active sounds",
                        value=str(self.active_sounds)))
                ds.values.append(
                    KeyValue(
                        key="Allocated sound channels",
                        value=str(self.num_channels)))
                ds.values.append(
                    KeyValue(
                        key="Buffered builtin sounds",
                        value=str(len(self.builtinsounds))))
                ds.values.append(
                    KeyValue(
                        key="Buffered wave sounds",
                        value=str(len(self.filesounds))))
                ds.values.append(
                    KeyValue(
                        key="Buffered voice sounds",
                        value=str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device." +\
                    "See http://wiki.ros.org/sound_play/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = self.get_clock().now().to_msg()
            self.diagnostic_pub.publish(da)
        except Exception as e:
            self.get_logger().error('Exception in diagnostics: %s' % str(e))

    def execute_cb(self, goal_handle):
        data = goal_handle.request.sound_request
        if not self.initialized:
            self.get_logger().error('soundplay_node is not initialized yet.')
            goal_handle.abort()
            return

        with self.mutex:
            # Force only one sound at a time
            self.stopall()
            result = SoundRequestAction.Result()
            try:
                if (data.sound == SoundRequest.ALL
                        and data.command == SoundRequest.PLAY_STOP):
                    self.stopall()
                else:
                    sound = self.select_sound(data)
                    sound.command(data.command)

                    start_time = self.get_clock().now()
                    success = True
                    while sound.get_playing():
                        sound.update()
                        if not goal_handle.is_active:
                            self.get_logger().info(
                                'sound_play action: Preempted')
                            sound.stop()
                            success = False
                            break
                        feedback = SoundRequestAction.Feedback()
                        feedback.playing = sound.get_playing()
                        feedback.stamp = (
                            self.get_clock().now() - start_time).to_msg()
                        goal_handle.publish_feedback(feedback)
                        self.sleep(1.0 / self.loop_rate)
                    if success:
                        result.playing = feedback.playing
                        result.stamp = feedback.stamp
                        self.get_logger().info('sound_play action: Succeeded')
                        goal_handle.succeed()
            except Exception as e:
                goal_handle.abort()
                self.get_logger().error(
                    'Exception in actionlib callback: %s' % str(e))
                self.get_logger().info(traceback.format_exc())
            finally:
                self.get_logger().debug("done actionlib callback")
        return result

    def handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def init_vars(self):
        self.num_channels = 10
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            self.get_logger().info('sound_play node is ready to play sound')

    def sleep(self, duration):
        time.sleep(duration)

    def get_sound_length(self):
        sound_length = len(self.builtinsounds) +\
            len(self.voicesounds) + len(self.filesounds)
        return sound_length

    def idle_loop(self):
        self.last_activity_time = self.get_clock().now()
        while (self.get_sound_length() > 0 and rclpy.ok()):
            loop_time = self.get_clock().now() - self.last_activity_time
            if loop_time > rclpy.duration.Duration(seconds=10):
                break
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()

if __name__ == '__main__':
    rclpy.init()
    soundplay_node = SoundPlayNode()
    rclpy.spin(soundplay_node)
    soundplay_node.destroy_node()
    del soundplay_node
    rclpy.shutdown()
