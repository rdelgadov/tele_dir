#!/usr/bin/env python

import rospy
import std_msgs.msg

## There are some important auxiliary functions in this file.

def mpe_dict(msg):
    for k, v in msg.items():
        if isinstance(v, dict):
            print k + ':'
            v = message_param_editor(v)
        else:
            v_type = type(v)
            v = raw_input(k + ': ')
            while True:
                try:
                    v = eval("v_type(" + v + ")")
                    break
                except:
                    print "type error, you input type is :" + type(v).__name__ + " enter " + v_type.__name__ + " :"
                    v = raw_input(k + ": ")
        msg[k] = v

def mpe_list(msg):
    for v in msg:
        if hasattr(v, '__iter__'):
            print type(v) + ':'
            value = message_param_editor(v)
        else:
            v_type = type(v)
            value = raw_input(v_type + ':')
            while True:
                try:
                    v = eval("v_type(" + v + ")")
                    break
                except:
                    print "type error, you input type is :" + type(v).__name__ + " enter " + v_type.__name__ + " :"
                    value = raw_input(v_type + ": ")
        msg[v] = value

def message_param_editor(msg):
    '''
    :param msg: A dictionary or a list that describes a message.
    :return obj: The same modified dictionary or list to fit to the modifications of the user.
    Prompts the user about the new parameters for an existing message.
    '''
    if isinstance(msg, dict):
        mpe_dict(msg)
    elif isinstance(msg, list):
        mpe_list(msg)
#    elif isinstance(obj, list):
#        for v in obj:
#            if hasattr(v, '__iter__'):
#                dumpclean(v)
#            else:
#                print v
    return msg


def our_raw_input(string, *args):
    valid_response = False
    while not valid_response:
        response = raw_input(string)
        if response.upper() == 'QUIT':
            raise ValueError("Quit!")
        for option in args:
            if option == response.upper():
                valid_response = True
        if not valid_response:
            print "-> Invalid response."
    return response

def message_raw_input(string):
    import roslib.message
    topic_msg = raw_input(string)
    while True:
        try:
            if roslib.message.get_message_class(topic_msg) == None:
                raise ValueError()
            break
        except ValueError as e:
            topic_msg = raw_input("The message type is invalid."
                                  " Enter a valid type: ")
    return topic_msg

