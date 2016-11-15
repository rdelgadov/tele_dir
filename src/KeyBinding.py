#!/usr/bin/env python
# This class handles key bindings. It is used to both hold the data structure of key bindings, and publishing the
# messages bound to the keys to the bound subscribed topic.

import roslib
import rospy

class KeyBinding:
    '''
    This is the data structure that supports key binding information. It holds a relation between a button,
    a message, and a topic. This key will eventually be used to send the desired message to the corresponding topic.
    '''
    _key = ''
    _message = None
    _subscribed_topic = ''

    def __init__( self, key, message, subscribed_topic):
        '''
        :param key: A keyboard button.
        :param message: A ROS message ready to be published.
        :param subscribed_topic: A ROS topic open for message publication.
        This is the basic constructor method for the KeyBdingind class.
        '''
        self._key = key
        self._message_class = message
        self._subscribed_topic = subscribed_topic

    def get_key(self):
        '''
        :return self._key: A string representing the key to this binding.
        This method allows public access to the key of the present binding.
        '''
        return self._key

    def get_message(self):
        '''
        :return self._message: A string representing the bound message.
        This method allows public access to the message content.
        '''
        return self._message

    def get_subscribed_topic(self):
        '''
        :return self._subscribed_topic: A string representing the bound subscribed topic.
        This method allows public access to the subscribed topic.
        '''
        return self._subscribed_topic

    def publish_message(self):
        '''
        :return None:
        This method publishes the bound message to the subscribed topic.
        '''
        pub = rospy.Publisher(self._subscribed_topic,roslib.message.get_message_class(self._message),queue_size=10)
        pub.publish(self._message)