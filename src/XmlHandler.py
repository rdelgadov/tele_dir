#!/usr/bin/env python

import AuxFuns
import roslib.message
import xml.etree.ElementTree as ET
import yaml
import json

from rospy_message_converter import message_converter
from AuxFuns import our_raw_input
from AuxFuns import message_raw_input

def validator(tree):
    configuration = tree.getroot().find("config")
    if configuration == None:
        print "This config file lacks the config tag."
        return False

    buttonKeyDict = {}
    idMessagesDict = {}
    idTopicDict = {}

    ## We check that there are lists for buttons, messages and topics.
    buttons = configuration.find("buttons")
    if buttons == None:
        print "This config file lacks the buttons tag."
        return False

    messages = configuration.find("messages")
    if messages == None:
        print "This config file lacks the messages tag."
        return False

    topics = configuration.find("topics")
    if topics == None:
        print "This config file lacks the topics tag."
        return False

    ## And procceed to check the integrity of all of them.
    for message in messages:
        try:
            if not idMessagesDict.has_key(message.attrib["id"]):
                idMessagesDict.setdefault(message.attrib["id"], 0)
            else:
                raise ValueError('The message id "' + message.attrib["id"] + '" is repeated.')
        except ValueError as e:
            print e.message
            return False

        try:
            if roslib.message.get_message_class(message.find("type").text) == None:
                raise ValueError('The type message is not a valid type')
            else:
                the_message = roslib.message.get_message_class(message.find("type").text)
                message_object = message_converter.convert_dictionary_to_ros_message(message.find("type").text,
                                                                                     json.loads(
                                                                                         message.find("content").text))

        except ValueError as e:
            print e.message
            return False
    for topic in topics:
        try:
            if not idTopicDict.has_key(topic.attrib["id"]):
                idTopicDict.setdefault(topic.attrib["id"], 0)
            else:
                raise ValueError('The topic id "' + topic.attrib["id"] + '" is repeated.')
        except ValueError as e:
            print e.message
            return False

    ## We finally check the integrity of the buttons, deciding if the messages sent are compatible
    ## with the receiving topic.
    for button in buttons:
        try:
            if not buttonKeyDict.has_key(button.find("key").text.upper()):
                buttonKeyDict.setdefault(button.find("key").text.upper(), 0)
            else:
                raise ValueError('The "' + button.find("key").text.upper() + '" key is repeated.')
        except ValueError as e:
            print e.message
            return False
        ## This part checks that the message type is compatible with the topic's expectations.
        ismessage = False
        istopic = False
        try:
            for message in messages:
                if button.find("message").text == message.attrib["id"]:
                    ismessage = True
                    for topic in topics:
                        if button.find("topic").text == topic.attrib["id"]:
                            istopic = True
                            if not topic.find("msg_type").text == message.find("type").text:
                                raise ValueError(
                                    'The message ' + message.attrib["id"] + ' is not compatible with the topic ' +
                                    topic.attrib["id"] + '\'s type.')
            if not ismessage or not istopic:
                raise ValueError('The ' + button.find(
                    "key").text.upper() + ' key\'s associated message type is not compatible with it\'s associated topic.')
        except ValueError as e:
            print e.message
            return False
    return True

def xml_validator(xml):
    '''
    :param xml: An XML file.
    :return Boolean: It's true if the XML file is a valid XML configuration file.
    :raises ValueError: It raises an exception if the XML file is not a valid configuration file.
    This function checks that the input XML file is a valid XML configuration file for tele-dir.
    If the file is not valid, then the program ends and raises an error.
    '''

    ## We first check if the XML file is a valid XML file.
    try:
        tree =  ET.parse(xml)
    except ET.ParseError:
        print xml+" is not a valid XML file."
        return False

    ## We then check that it has a configuration section.
    return validator(tree)


def xmlCreator(xml_dir):
    '''
    :return None:
    This function starts a prompt in the terminal for the user to create a custom valid XML configuration file.
    It asks for several inputs for the user to fill with the information required to make the configuration that
    the user desires.
    '''
    master = ET.ElementTree()
    xml = ET.Element("xml")
    master._setroot(xml)
    description = ET.Element("description")
    config = ET.Element("config")
    messages = ET.Element("messages")
    buttons = ET.Element("buttons")
    topics = ET.Element("topics")
    print "Initializing controller configuration."
    file_name = raw_input("Input file name: ")
    while len(file_name) < 1:
        file_name = raw_input("Input file name: ")
    name = ET.Element("name")
    name.text = file_name
    robot_name = raw_input("Input target robot name: ")
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
    while True:
        topic = newTopic(i)
        topics.insert(i - 1, topic)
        i += 1
        end = our_raw_input("Do you wish to add another topic? (Y/N)", 'Y', 'N')
        if end.upper() == 'N':
            break
    i = 1
    print "Initializing messages configuration. "
    while True:
        message = newMessage(i)
        messages.insert(i - 1, message)
        i += 1
        end = our_raw_input("Do you wish to add another message? (Y/N)", 'Y', 'N')
        if end.upper() == 'N':
            break
    i = 1
    print "Initializing keyboard configuration. "
    while True:

        button = newButton(i, topics, messages)
        buttons.insert(i - 1, button)
        i += 1
        end = our_raw_input("Do you wish to add another button? (Y/N)", 'Y', 'N')
        if end.upper() == 'N':
            break
    config.insert(1, messages)
    config.insert(2, topics)
    config.insert(3, buttons)
    xml.insert(1, config)
    master.write(xml_dir + file_name + ".xml")
    print "File Created with name :" + file_name + ".xml!"


def delete_key_by_topic(topic, buttons):
    list = []
    for button in buttons:
        if button.find("topic").text == topic.attrib['id']:
            list.append(button)
            buttons.remove(button)
    if len(list) > 0:
        print "The following buttons have been deleted: ",
        for button in list:
            print button.find("key").text + " ",
        print ".\n"
    else :
        print "No key has been deleted due to topic removal."
    return list


def delete_key_by_message(message, buttons):
    list = []
    for button in buttons:
        if button.find("message").text == message.attrib['id']:
            list.append(button)
            buttons.remove(button)
    if len(list) > 0:
        print "The following buttons have been deleted: ",
        for button in list:
            print button.find("key").text + " ",
        print ".\n"
    else:
        print "No key has been deleted due to message removal."
    return list


def newMessage(i):
    message_description = raw_input("Input message description: ")
    message_type = message_raw_input("Input message type: ")
    message_class = roslib.message.get_message_class(message_type)
    message_body = message_converter.convert_ros_message_to_dictionary(eval("message_class()"))
    message_content = json.dumps(AuxFuns.message_param_editor(message_body), sort_keys=True)
    return createMessage(i, message_description, message_type, message_content)


def createMessage(i, message_description, message_type, message_content):
    content = ET.Element("content")
    content.text = message_content
    message = ET.Element("message", {'id': str(i)})
    description = ET.Element("description")
    description.text = message_description
    type = ET.Element("type")
    type.text = message_type
    message.insert(0, description)
    message.insert(1, type)
    message.insert(2, content)
    return message

def newTopic(i):
    topic_name = raw_input("Input topic name: ")
    topic_msg = message_raw_input("Input topic msg_type: ")
    return createTopic(i, topic_name, topic_msg)

def createTopic(i, topic_name, topic_msg):
    topic = ET.Element("topic", {'id': str(i)})
    name = ET.Element("name")
    name.text = topic_name
    msg_type = ET.Element("msg_type")
    msg_type.text = topic_msg
    topic.insert(0, name)
    topic.insert(1, msg_type)
    return topic

def newButton(i, topics, messages):
    button_key = raw_input("Input only one key: ")
    while len(button_key) > 1:
        button_key = raw_input("Error Length " + str(len(button_key)) + ".Input only one key:")
    message_asociated = None
    while message_asociated == None:
        button_messages = raw_input("Input number of message associated: ")
        for message in messages:
            if button_messages == message.attrib['id']:
                message_asociated = message
        if message_asociated == None:
            print "Error, the message wasn't found"
    topic_associated = None
    while topic_associated == None:
        button_topics = raw_input("Input topic associated: ")
        if len(topics.findall("topic")) < 1:
            raise ValueError("Topics is empty. Can't associate with a message")
        for topic in topics:
            if button_topics == topic.attrib['id']:
                topic_associated = topic
        if topic_associated!= None:
            if message_asociated.find("type").text != topic_associated.find("msg_type").text:
                print "Error message type in the topic selected (" + topic_associated.find("msg_type").text + ") , " \
                      "isn't the same that the message type selected (" + message_asociated.find("type").text +")."
                topic_associated = None
        else:
            print "Topic with number " + button_topics + " don't exist. The existing topics are :",
            for topic in topics:
                print " " + topic.attrib['id'],
            print "\n"
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
    return button

def xmlEditor(xmlUrl, xml):
    '''
    :param xml: A valid tele-dir XML configuration file.
    :return None:
    This function prompts the user for changes in the xml XML file. It's designed to further improve the customization options for the user.
    '''
    #TODO all this shit

    try:
        tree =  ET.parse(xmlUrl+xml)
    except ET.ParseError:
        print xml+" is not a valid XML file."
        return False


    edited = False
    print "Welcome to the tele-dir configuration editor. You are our user 1.000.000 ! You have won a prize :) ask Johan Fabry for it."
    descr_edit = our_raw_input("Do you wish to edit the description? (Y/N)", 'Y', 'N')
    if descr_edit.upper() == 'Y':
        description = tree.getroot().find("description")
        name = description.find("name")
        target_robot = description.find("target_robot")
        print   "The current name for this configuration is '"+name.text+"'" \
                "And the target robot is '"+target_robot.text+"'."
        edit = our_raw_input("Input N to edit the name\n"
                            "      R to edit the target robot\n"
                            "      F to forget about editing the description and move forward\n", 'R', 'F', 'N')
        if edit.upper() == 'N':
            new_name = raw_input("Input new name: ")
            name.text = new_name
            edit = our_raw_input("Do yo wish to edit the target robot? (Y/N)", 'Y', 'N')
        if edit.upper() == 'R' or edit.upper() == 'Y':
            new_target_robot = raw_input("Input new target robot: ")
            target_robot.text = new_target_robot

    conf_edit = our_raw_input("Do you wish to edit the configuration? (Y/N)", 'Y', 'N')
    if conf_edit.upper() == 'Y':
        configuration = tree.getroot().find("config")
        while True:

            buttons = configuration.find("buttons")
            messages = configuration.find("messages")
            topics = configuration.find("topics")
            edit = our_raw_input("Input T to edit topics\n"
                             "      M to edit messages\n"
                         "      B to edit buttons\n", 'T', 'M', 'B').upper()
            if edit == 'T':
                topic_bool = our_raw_input("Do you wish yo Add or Modify topics? (A/M)", 'A', 'M').upper()
                if topic_bool == 'M':
                    for topic in topics:
                        print "The topic number "+topic.attrib['id']+" is: "
                        print ET.dump(topic)
                        option = our_raw_input("Input E to edit the topic \n"
                                              "      D to delete the topic \n"
                                              "      C to continue to the next topic.\n", 'E', 'D', 'C').upper()
                        if option == 'E':
                            topic.find("name").text = raw_input("Input topic name: ") or topic.find("name").text
                            topic.find("msg_type").text = message_raw_input("Input message type: ")
                        elif option == 'D':
                            if our_raw_input("Are you sure? (Y/N)", 'Y', 'N').upper() == 'Y':
                                delete_key_by_topic(topic, buttons)
                                topics.remove(topic)
                elif topic_bool == 'A':
                    while True:
                        i = int(topics.findall('topic')[len(topics.findall('topic'))-1].attrib['id'])+1
                        topic = newTopic(i)
                        topics.insert(i - 1, topic)
                        end = our_raw_input("Do you wish to add another topic? (Y/N)", 'Y', 'N')
                        if end.upper() == 'N':
                            break
            elif edit == 'M':
                mess_bool = our_raw_input("Do you wish yo Add or Modify messages? (A/M)", 'A', 'M').upper()
                if mess_bool == 'M':
                    for message in messages:
                        print "The message with id "+message.attrib['id']+" is: "
                        print ET.dump(message)
                        option = our_raw_input("Input E to edit the message \n"
                                              "      D to delete the message \n"
                                              "      C to continue to the next message.\n", 'E', 'D', 'C').upper()
                        if option == 'E':
                            message.find("description").text = raw_input("Input message description: ") or message.find("description").text
                            type = message_raw_input("Input message type: ")
                            if message.find("type").text == type:
                                message.find("content").text = json.dumps(AuxFuns.message_param_editor(json.loads(message.find("content").text)),sort_keys=True)
                            else:
                                message_class = roslib.message.get_message_class(type)
                                message_body = message_converter.convert_ros_message_to_dictionary(eval("message_class()"))
                                message.find("content").text = json.dumps(AuxFuns.message_param_editor(message_body),sort_keys=True)
                        elif option == 'D':
                            if our_raw_input("Are you sure? (Y/N)", 'Y', 'N').upper() == 'Y':
                                delete_key_by_message(message, buttons)
                                messages.remove(message)

                elif mess_bool == 'A':
                    while True:
                        i = int(messages.findall('message')[len(messages.findall('message'))-1].attrib['id'])+1
                        message = newMessage(i)
                        messages.insert(i - 1, message)
                        end = our_raw_input("Do you wish to add another message? (Y/N)", 'Y', 'N')
                        if end.upper() == 'N':
                            break
            elif edit == 'B':
                butt_bool = our_raw_input("Do you wish yo Add or Modify buttons? (A/M)", 'A', 'M').upper()
                if butt_bool == 'M':
                    i = 0
                    for button in buttons:
                        print "The button number "+i+" is: "
                        print ET.dump(button)
                        i+=1
                        option = our_raw_input("Input E to edit the button \n"
                                              "      D to delete the button \n"
                                              "      C to continue to the next button.\n", 'E', 'D', 'C').upper()
                        if option == 'E':
                            button_key = raw_input("Input button key") or button.find("key").text
                            while len(button_key) > 1:
                                button_key = raw_input("Error Length " + str(len(button_key)) + ".Input only one key:")or button.find("key").text
                            message_asociated = None
                            while message_asociated == None:
                                button_messages = raw_input("Input number of message associated: ") or button.find("message").text
                                for message in messages:
                                    if button_messages == message.attrib['id']:
                                        message_asociated = message
                                if message_asociated == None:
                                    print "Error, the message wasn't found"
                            topic_associated = None
                            while topic_associated == None:
                                button_topics = raw_input("Input topic associated: ") or button.find("topic").text
                                for topic in topics:
                                    if button_topics == topic.attrib['id']:
                                        topic_associated = topic
                                if message_asociated.find("type").text != topic_associated.find("msg_type").text:
                                    print "Error message type in the topic selected, isn't the same that the message type selected."
                                    topic_associated = None
                            ## TODO: add a prompt to confirm the message/topic selections, displaying said messages and or topics
                            button.find("topic").text = button_topics
                            button.find("key").text = button_key
                            button.find("message").text = button_messages
                        elif option == 'D':
                            if our_raw_input("Are you sure? (Y/N)", 'Y', 'N').upper() == 'Y':
                                buttons.remove(button)
                elif butt_bool == 'A':
                    while True:
                        try:
                            i = len(buttons.findall('button'))+1
                            button = newButton(i, topics, messages)
                            buttons.insert(i - 1, button)
                            end = our_raw_input("Do you wish to add another button? (Y/N)", 'Y', 'N')
                            if end.upper() == 'N':
                                break
                        except ValueError as e :
                            print e.message
                            break

            if our_raw_input("Do you wish continue editing? (Y/N)", 'Y', 'N').upper() == 'N':
                break

    file = raw_input("Enter the new file name: ")
    file_name = xmlUrl + file + ".xml"
    if (file== ""):
        file_name = xmlUrl + xml
    tree.write(file_name )
    print "The file has been saved with name "+ file_name






