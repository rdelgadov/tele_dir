#!/usr/bin/env python
from types import NoneType

import ast
import genpy
import rosgraph
import rospy
import roslib.message

import rostopic as rostop
import select, sys, tty, termios
import xml.etree.ElementTree as ET
import yaml
import std_msgs.msg
from geometry_msgs.msg import Twist
from rospy_message_converter import message_converter

import xmlvalidator

global old_attr

def tele_dir(config):
    tree = ET.parse(config)
    configuration = tree.getroot().find("config")
    buttons = []
    keyboard = {}
    messages = []
    topics = []
    lspeed = 1
    aspeed = 1
    for topic in configuration.find("topics"):
        topics.append(topic)
    for button in configuration.find("buttons"):
        buttons.append(button)
    for message in configuration.find("messages"):
        messages.append(message)
    for button in buttons:
        keyboard.setdefault( button.find('key').text,
                            ( button.find("message").text,
                              button.find("topic").text ) )
    rospy.init_node('tele_dir', anonymous=True)
    rate = rospy.Rate(200)  # 10hz
    global old_attr

    print "Use '+' and '-' to modify linear speed. \n" \
          "Use '*' and '/' to modify angular speed. \n" \
          "Press '.' to exit. \n" \
          "Publishing Keystrokes"
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            input = sys.stdin.read(1).upper()
            if input == '+':
                lspeed += 0.5
                print "Linear speed set to", lspeed
            elif input == '-':
                if lspeed >= 0.5:
                    lspeed -= 0.5
                else:
                    lspeed = 0.0
                print "Linear speed set to", lspeed
            elif input == '*':
                aspeed += 0.5
                print "Angular speed set to", aspeed
            elif input == '/':
                if aspeed >= 0.5:
                    aspeed -= 0.5
                else:
                    aspeed = 0.0
                print "Angular speed set to", aspeed

            elif keyboard.has_key(input):
                linear_velocity = messages[int(keyboard.get(input)[0])-1].find("content").find("linear")
                angular_velocity = messages[int(keyboard.get(input)[0])-1].find("content").find("angular")
                mess = Twist()
                mess.linear.x = float(linear_velocity.find("x").text)*lspeed
                mess.linear.y = float(linear_velocity.find("y").text)*lspeed
                mess.linear.z = float(linear_velocity.find("z").text)*lspeed
                mess.angular.x= float(angular_velocity.find("x").text)*aspeed
                mess.angular.y = float(angular_velocity.find("y").text)*aspeed
                mess.angular.z = float(angular_velocity.find("z").text)*aspeed
                pub = rospy.Publisher(topics[int(keyboard.get(input)[1])-1].find("name").text, roslib.message.get_message_class(messages[int(keyboard.get(input)[0])-1].find("type").text), queue_size=10)
                rospy.init_node('tele_dir', anonymous=True)
                pub.publish(mess)
                rate.sleep()
            elif input == '.':
                break
            else:
                print "unrecognized input"
                print input
def xmlCreator():
    master = ET.ElementTree()
    xml = ET.Element("xml")
    master._setroot(xml)
    description = ET.Element("description")
    config = ET.Element("config")
    messages = ET.Element("messages")
    buttons = ET.Element("buttons")
    topics = ET.Element("topics")
    print "Initializing controller configuration."
    file_name = raw_input( "Input file name: " )
    while len(file_name)<1:
        file_name = raw_input("Input file name: ")
    name = ET.Element("name")
    name.text = file_name
    robot_name = raw_input( "Input target robot name: ")
    robot = ET.Element("target_robot")
    robot.text = robot_name
    config_version = ET.Element("config_version")
    config_version.text = "1.0"
    description.insert(0, name)
    description.insert(1, robot)
    description.insert(2, config_version)
    xml.insert(0, description)
    print "Initializing topics configuration. "
    i = 1
    while True :
        topic_name = raw_input("Input topic name: ")
        topic_msg = raw_input("Input topic msg_type: ")
        while True :
            try:
                roslib.message.get_message_class(topic_msg)
                break
            except ValueError as e:
                topic_msg = raw_input("The message type is invalid."
                                      " Enter a valid type: ")
        topic = ET.Element("topic", {'id': str(i) })
        name = ET.Element("name")
        name.text = topic_name
        msg_type = ET.Element("msg_type")
        msg_type.text = topic_msg
        topic.insert(0, name)
        topic.insert(1, msg_type)
        topics.insert(i-1, topic)
        i+=1
        end = raw_input( " Do you wish to add another topic? (Y/N) " )
        while end.upper() != 'Y' and end.upper() != 'N':
            end = raw_input(" Invalid Key. Do you wish to add another topic? (Y/N) ")
        if end.upper() == 'N':
            break
    i = 1
    print "Initializing messages configuration. "
    while True :
        message_description = raw_input("Input message description: ")
        message_type = raw_input("Input message type: ")
        while True:
            try:
                if roslib.message.get_message_class(message_type)==None:
                    raise ValueError()
                break
            except ValueError as e:
                message_type = raw_input("The message type is invalid."
                                      " Enter a valid type: ")
        message_class = roslib.message.get_message_class(message_type)
        message_body = message_converter.convert_ros_message_to_dictionary(eval("message_class()"))
        content = ET.Element("content")
        content.text= yaml.dump(dumpclean(message_body))
        message = ET.Element("message", {'id': str(i)})
        description = ET.Element("description")
        description.text = message_description
        type = ET.Element("type")
        type.text = message_type
        message.insert(0,description)
        message.insert(1,type)
        message.insert(2,content)
        messages.insert(i-1,message)
        i+=1
        end = raw_input(" Do you wish to add another message? (Y/N) ")
        while end.upper() != 'Y' and end.upper() != 'N':
            end = raw_input(" Invalid Option. Do you wish to add another message? (Y/N) ")
        if end.upper() == 'N':
            break
    i=1
    print "Initializing keyboard configuration. "
    while True:
        button_key = raw_input("Input only one key: ")
        while len(button_key) > 1:
            button_key = raw_input("Error Length "+ str(len(button_key)) +".Input only one key:")
        message_asociated = None
        while message_asociated == None:
            button_messages = raw_input("Input number of message associated: ")
            for message in messages:
                if button_messages == message.attrib['id']:
                    message_asociated = message
            if message_asociated==None:
                print "Error, the message wasn't found"
        topic_associated = None
        while topic_associated == None:
            button_topics = raw_input("Input topic associated: ")
            for topic in topics:
                if button_topics == topic.attrib['id']:
                    topic_associated = topic
            if message_asociated.find("type").text!=topic_associated.find("msg_type").text:
                print "Error message type in the topic selected, isn't the same that the message type selected."
                topic_associated = None
        key = ET.Element("key")
        message = ET.Element("message")
        topic = ET.Element("topic")
        key.text = button_key.upper()
        message.text = button_messages
        topic.text = button_topics
        button = ET.Element("button")
        button.insert(0, key)
        button.insert(1, message)
        button.insert(2, topic)
        buttons.insert(i-1,button)
        i+=1
        end = raw_input(" Do you wish to add another button? (Y/N) ")
        while end.upper() != 'Y' and end.upper() != 'N':
            end = raw_input(" Invalid Option. Do you wish to add another button? (Y/N) ")
        if end.upper() == 'N':
            break

    ET.dump(messages)
    config.insert(1, messages)
    config.insert(2, topics)
    config.insert(3, buttons)
    ET.dump(config)
    xml.insert(1, config)
    master.write("Configs/"+file_name+".xml")
    print "File Created with name :" + file_name + ".xml!"


def _fillMessageArgs(msg, pub_args):
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

###
#####  asdasd
#def MessageParser(type,slot):
 #   node = ET.Element(slot)
  #  try:
   #     i = 0
    #    for i in type :
     #       node.insert( i, MessageParser(camp, ))
#####
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
## main maneja el menu principal y las funcionalidades a llamar
if __name__ == '__main__':
    old_attr = termios.tcgetattr(sys.stdin)
    try:

        print "Welcome to Tele_Dir ! \n" \
              "Press C to launch the configuration wizard. \n" \
              "Press L to load a custom configuration (XML).\n" \
              "Press D to use the default configuration .\n" \
              "Press E to exit. \n " \
              ">"
        while True:
            tty.setcbreak(sys.stdin.fileno())

            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                input = sys.stdin.read(1).upper()

                if input.upper()=='C' :
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    xmlCreator()
                    tty.setcbreak(sys.stdin.fileno())
                elif input.upper() == 'L':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    xmlLoad = raw_input("Input the filename: ")
                    xmlvalidator.xmlvalidator("Configs/"+xmlLoad)
                    tty.setcbreak(sys.stdin.fileno())
                    tele_dir("Configs/"+xmlLoad)

                elif input.upper() == 'D':
                    tele_dir("Configs/default_config.xml")
                elif input.upper() == 'E':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    exit(0)
                else:
                    print ">"+input
    except rospy.ROSInterruptException:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


# vector = rosgraph.names.script_resolve_name('rostopic', "/rosout")
 #       type = "nav_msgs/Odometry"
#
 #       msg_class = roslib.message.get_message_class(type)
  #      print len(msg_class.__slots__)
   #     asd = """
    #    linear :
    #         x : 1.0
     #        y : 1.0
      #       z : 1.0
       # angular :
        #     x : 0.0
         #    y : 0.0
          #   z : 1.0
        #"""
        #cars = message_converter.convert_ros_message_to_dictionary(eval("msg_class()"))
        #dumpclean(cars)

        #_fillMessageArgs(msg_class,(yaml.load(asd)))

####

