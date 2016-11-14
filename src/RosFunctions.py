#!/usr/bin/env python

import genpy
import rospy
import std_msgs.msg

def file_yaml_arg(filename):
    """
    :param filename: file name, ``str``
    :returns: Iterator that yields pub args (list of args), ``iterator``
    :raises: :exc:`ROSTopicException` If filename is invalid
    """
    if not os.path.isfile(filename):
        raise ValueError("file does not exist: %s"%(filename))
    import yaml
    def bagy_iter():
        try:
            with open(filename, 'r') as f:
                # load all documents
                data = yaml.load_all(f)
                for d in data:
                    yield [d]
        except yaml.YAMLError as e:
            raise ValueError("invalid YAML in file: %s"%(str(e)))
    return bagy_iter

def fillMessageArgs(msg, pub_args):
    try:
        # Populate the message and enable substitution keys for 'now'
        # and 'auto'. There is a corner case here: this logic doesn't
        # work if you're publishing a Header only and wish to use
        # 'auto' with it. This isn't a troubling case, but if we start
        # allowing more keys in the future, it could become an actual
        # use case. It greatly complicates logic because we'll have to
        # do more reasoning over types. to avoid ambiguous cases
        # (e.g. a std_msgs/String type, which only has a single string
        # field).

        # allow the use of the 'now' string with timestamps and 'auto' with header

        now = rospy.get_rostime()
        print pub_args
        keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        print pub_args
        genpy.message.fill_message_args(msg, pub_args, keys=keys)
    except genpy.MessageException as e:
        raise ValueError(str(e)+"\n\nArgs are: [%s]"%genpy.message.get_printable_message_args(msg))


def dumpclean(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(v,dict):
                print k +':'
                v = dumpclean(v)
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
            obj[k] = v
#    elif isinstance(obj, list):
#        for v in obj:
#            if hasattr(v, '__iter__'):
#                dumpclean(v)
#            else:
#                print v
    return obj