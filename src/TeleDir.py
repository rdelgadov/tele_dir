#!/usr/bin/env python
# Main tele-dir program.
import yaml

from rospy_message_converter import message_converter

import AuxFuns
import rospy
from KeyBinding import KeyBinding
import select, sys, tty, termios
import xml.etree.ElementTree as ET
import XmlHandler
import rospkg
import os
import json



global old_attr

def tele_dir( config ):
    '''
    :param config:  A valid XML that holds information about what messages to be sent to what
                    topic when which button is pressed.
    :return None:
    This function's purpose is to handle the button inputs as the config parameter states. That is, when a key is
    pressed this will send the corresponding message to the corresponding topic.
    '''

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
        message_id = button.find("message").text
        topic_id = button.find("topic").text
        topic_name = ''
        for message in messages:
            if message.attrib["id"]==message_id:
                message_class = message.find('type').text
                message_content = message.find('content').text
                message_info = message.find('description').text
        for topic in topics:
            if topic.attrib["id"] == topic_id:
                topic_name = topic.find("name").text
        message = message_converter.convert_dictionary_to_ros_message(message_class,json.loads(message_content))
        key = KeyBinding(button.find('key').text, message, topic_name, message_info)
        keyboard.setdefault( button.find('key').text, key )

    rate = rospy.Rate(60)  # 200hz
    global old_attr

    print "Press '.' to exit. \n" \
          "Publishing Keystrokes \n"
    for keys in keyboard.values():
        print keys.get_key() + " " + keys.get_info() + ".\n"

    last_input=''
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0.1)[0] == [sys.stdin]:
            input = sys.stdin.read(1).upper()
            # if input == '+':
            #     lspeed += 0.5
            #     print "Linear speed set to", lspeed
            # elif input == '-':
            #     if lspeed >= 0.5:
            #         lspeed -= 0.5
            #     else:
            #         lspeed = 0.0
            #     print "Linear speed set to", lspeed
            # elif input == '*':
            #     aspeed += 0.5
            #     print "Angular speed set to", aspeed
            # elif input == '/':
            #     if aspeed >= 0.5:
            #         aspeed -= 0.5
            #     else:
            #         aspeed = 0.0
            #     print "Angular speed set to", aspeed

            if input in keyboard:
                publish = keyboard.get(input)
                publish.publish_message()
                if last_input!=input:
                    rospy.logdebug(publish.info)
                    last_input = input

                rate.sleep()
            elif input == '.':
                break
            else:
                print "unrecognized input"
                print input



## main maneja el menu principal y las funcionalidades a llamar
if __name__ == '__main__':
    xml_dir = rospkg.RosPack().get_path('tele_dir') + "/src/Configs/"
    old_attr = termios.tcgetattr(sys.stdin)
    rospy.init_node('tele_dir', anonymous=True)
    try:

        print "Welcome to Tele_Dir ! \n" \
              "Press C to launch the configuration wizard. \n" \
              "Press D to use the default configuration .\n" \
              "Press L to load a custom configuration (XML).\n" \
              "Press E to edit a custom configuration (XML). \n" \
              "Press Q to quit. \n "
        while True:
            tty.setcbreak(sys.stdin.fileno())

            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                input = sys.stdin.read(1).upper()
                print ">" + input
                if input.upper()=='C' :
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    XmlHandler.xmlCreator(xml_dir)
                    tty.setcbreak(sys.stdin.fileno())

                elif input.upper() == 'L':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    print "configuraciones:" #TODO:Escribirlo en ingles !!!
                    for file in os.listdir(xml_dir):
                        if file.endswith(".xml"):

                            print(file.split(".")[0])
                    xmlLoad = raw_input("Input the filename: ")
                    try:
                        if XmlHandler.xml_validator( xml_dir + xmlLoad + ".xml"):
                            tty.setcbreak(sys.stdin.fileno())
                            tele_dir( xml_dir + xmlLoad + ".xml")
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    except:
                        print "file "+ xmlLoad + "don't exist."
                elif input.upper() == 'E':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    print "configuraciones:"  # TODO:Escribirlo en ingles !!!
                    for file in os.listdir(xml_dir):
                        if file.endswith(".xml"):
                            print(file.split(".")[0])
                    xmlLoad = raw_input("Input the filename: ")
                    try:
                        XmlHandler.xmlEditor(xml_dir , xmlLoad+".xml")
                    except:
                        print "file "+xmlLoad+ " don't exist."
                    tty.setcbreak(sys.stdin.fileno())

                elif input.upper() == 'D':
                    tele_dir(xml_dir + "default_config.xml")

                elif input.upper() == 'Q':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                    exit(0)



    except rospy.ROSInterruptException:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    except ValueError:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)



