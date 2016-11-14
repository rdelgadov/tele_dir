#!/usr/bin/env python

import roslib
import rospy



class KeyBinding:
    _key = ''
    _message = None
    _subscribed_topic = ''
    def __init__( self, key, message, subscribed_topic):
        self._key = key
        self._message_class = message
        self._subscribed_topic = subscribed_topic

    def get_key(self):
        return self._key

    def get_message(self):
        return self._message

    def get_subscribed_topic(self):
        return self._subscribed_topic

    def publish_message(self):
        pub = rospy.Publisher(self._subscribed_topic,roslib.message.get_message_class(self._message),queue_size=10)
        pub.publish(self._message)