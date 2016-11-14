#!/usr/bin/env python

import rospy
import roslib.message

import select, sys, tty, termios
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Twist

import XmlHandler

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
                    XmlHandler.xmlvalidator("Configs/" + xmlLoad)
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

