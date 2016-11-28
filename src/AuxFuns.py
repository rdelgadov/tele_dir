#!/usr/bin/env python

import genpy
import rospy
import std_msgs.msg

# def file_yaml_arg(filename):
#     """
#     :param filename: file name, ``str``
#     :returns: Iterator that yields pub args (list of args), ``iterator``
#     :raises: :exc:`ROSTopicException` If filename is invalid
#     """
#     if not os.path.isfile(filename):
#         raise ValueError("file does not exist: %s"%(filename))
#     import yaml
#     def bagy_iter():
#         try:
#             with open(filename, 'r') as f:
#                 # load all documents
#                 data = yaml.load_all(f)
#                 for d in data:
#                     yield [d]
#         except yaml.YAMLError as e:
#             raise ValueError("invalid YAML in file: %s"%(str(e)))
#     return bagy_iter

#copied from rostopic because it is a private overpowered method that
# should be callable from somewhere other than rostopic itself. we want an API.
def fill_message_args(msg, pub_args):
    '''
    :param msg: A ROS message instance.
    :param pub_args: A dictionary or a list with the message's parameters.
    :return None:
    This function populates the message and enable substitution keys
    for 'now' and 'auto'. There is a corner case here: this logic
    doesn't work if you're publishing a Header only and wish to use
    'auto' with it. This isn't a troubling case, but if we start
    allowing more keys in the future, it could become an actual
    use case. It greatly complicates logic because we'll have to
    do more reasoning over types. to avoid ambiguous cases
    (e.g. a std_msgs/String type, which only has a single string
    field).
    '''
    try:


        # allow the use of the 'now' string with timestamps and 'auto' with header

        now = rospy.get_rostime()
        keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        genpy.message.fill_message_args(msg, pub_args, keys=keys)
    except genpy.MessageException as e:
        raise ValueError(str(e)+"\n\nArgs are: [%s]"%genpy.message.get_printable_message_args(msg))


def message_param_editor(msg):
    '''
    :param msg: A dictionary that describes a message.
    :return obj: The same modified dictionary to fit to the modifications of the user.
    This function allows the user to change the contents of a message.
    '''
    if isinstance(msg, dict):
        for k, v in msg.items():
            if isinstance(v,dict):
                print k +':'
                v = message_param_editor(v)
            else:
                v_type = type(v)
                v = raw_input( k + ': ')
                while True:
                    try:
                        v = eval("v_type(" + v + ")")
                        break
                    except:
                        print "type error, you input type is :" + type(v).__name__ + " enter " + v_type.__name__ + " :"
                        v = raw_input(k + ": ")
            msg[k] = v
    elif isinstance(msg, list):
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
            msg[v]=value
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