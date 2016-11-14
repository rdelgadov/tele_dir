#!/usr/bin/env python

import RosFunctions
import roslib.message
import sys
import xml.etree.ElementTree as ET
import yaml

from rospy_message_converter import message_converter



def xmlvalidator(xml):
    tree = None
    try:
        tree =  ET.parse(xml)
    except ET.ParseError:
        print xml+" is not a valid XML file."
        exit(1)
    configuration = None
    description = None

    try:
        if tree.getroot().find("config") == None:
            raise ValueError("This config file lacks the config tag.")
        configuration = tree.getroot().find("config")
    except ValueError as e:
        print e.message
        exit(1)
    buttonKeyDict = {}
    buttonIdDict = {}
    idMessagesDict = {}
    idTopicDict= {}
    buttons = ()
    messages = ()
    topics = ()

    try:
        if configuration.find("buttons") == None:
            raise ValueError("This config file lacks the buttons tag.")
        buttons = configuration.find("buttons")
    except ValueError as e:
        print e.message
        exit(1)
    try:
        if configuration.find("messages") == None:
            raise ValueError("This config file lacks the messages tag.")
        messages=configuration.find("messages")
    except ValueError as e:
        print e.message
        exit(1)
    try:
        if configuration.find("topics") == None:
            raise ValueError("This config file lacks the topics tag.")
        topics = configuration.find("topics")
    except ValueError as e:
        print e.message
        exit(1)


    #Primero se verifica la integridad de los ID de mensajes y topicos
    for message in messages:
        try:
            if not idMessagesDict.has_key(message.attrib["id"]):
                idMessagesDict.setdefault(message.attrib["id"], 0)
            else:
                raise ValueError('The message id "' + message.attrib["id"] + '" is repeated.')
        except ValueError as e:
            print e.message
            exit(1)

        try:
            roslib.message.get_message_class(message.find("type").text)
        except ValueError as e:
            print e.message
            exit(1)
    for topic in topics:
        try:
            if not idTopicDict.has_key(topic.attrib["id"]):
                idTopicDict.setdefault(topic.attrib["id"], 0)
            else:
                raise ValueError('The topic id "'+topic.attrib["id"]+'" is repeated.' )
        except ValueError as e:
            print e.message
            exit(1)

    #A continuacion se verifica la integridad de los botones y sus mensajes relacionados y sus topicos.
    for button in buttons:
        try:
            if not buttonKeyDict.has_key(button.find("key").text.upper()):
                buttonKeyDict.setdefault(button.find("key").text.upper(), 0)
            else:
                raise ValueError('The "'+button.find("key").text.upper()+'" key is repeated.' )
        except ValueError as e:
            print e.message
            exit(1)
        #Luego se verifica que el mensaje que se envia con dicho boton sea del tipo que recibe el topico estipulado
        ismessage=False
        istopic=False
        try:
            for message in messages:
                if button.find("message").text == message.attrib["id"]:
                    ismessage = True
                    for topic in topics:
                        if button.find("topic").text == topic.attrib["id"]:
                            istopic = True
                            if not topic.find("msg_type").text==message.find("type").text:
                                raise ValueError('The message '+message.attrib["id"]+' is not compatible with the topic '+topic.attrib["id"]+'\'s type.')
            if not ismessage or not istopic:
                raise ValueError('The '+button.find("key").text.upper()+' key\'s associated message type is not compatible with it\'s associated topic.')
        except ValueError as e:
            print e.message
            exit(1)

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
        topic_name = raw_input("Input topic name: ")
        topic_msg = raw_input("Input topic msg_type: ")
        while True:
            try:
                roslib.message.get_message_class(topic_msg)
                break
            except ValueError as e:
                topic_msg = raw_input("The message type is invalid."
                                      " Enter a valid type: ")
        topic = ET.Element("topic", {'id': str(i)})
        name = ET.Element("name")
        name.text = topic_name
        msg_type = ET.Element("msg_type")
        msg_type.text = topic_msg
        topic.insert(0, name)
        topic.insert(1, msg_type)
        topics.insert(i - 1, topic)
        i += 1
        end = raw_input(" Do you wish to add another topic? (Y/N) ")
        while end.upper() != 'Y' and end.upper() != 'N':
            end = raw_input(" Invalid Key. Do you wish to add another topic? (Y/N) ")
        if end.upper() == 'N':
            break
    i = 1
    print "Initializing messages configuration. "
    while True:
        message_description = raw_input("Input message description: ")
        message_type = raw_input("Input message type: ")
        while True:
            try:
                if roslib.message.get_message_class(message_type) == None:
                    raise ValueError()
                break
            except ValueError as e:
                message_type = raw_input("The message type is invalid."
                                         " Enter a valid type: ")
        message_class = roslib.message.get_message_class(message_type)
        message_body = message_converter.convert_ros_message_to_dictionary(eval("message_class()"))
        content = ET.Element("content")
        content.text = yaml.dump(RosFunctions.dumpclean(message_body))
        message = ET.Element("message", {'id': str(i)})
        description = ET.Element("description")
        description.text = message_description
        type = ET.Element("type")
        type.text = message_type
        message.insert(0, description)
        message.insert(1, type)
        message.insert(2, content)
        messages.insert(i - 1, message)
        i += 1
        end = raw_input(" Do you wish to add another message? (Y/N) ")
        while end.upper() != 'Y' and end.upper() != 'N':
            end = raw_input(" Invalid Option. Do you wish to add another message? (Y/N) ")
        if end.upper() == 'N':
            break
    i = 1
    print "Initializing keyboard configuration. "
    while True:
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
            for topic in topics:
                if button_topics == topic.attrib['id']:
                    topic_associated = topic
            if message_asociated.find("type").text != topic_associated.find("msg_type").text:
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
        buttons.insert(i - 1, button)
        i += 1
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
    master.write("Configs/" + file_name + ".xml")
    print "File Created with name :" + file_name + ".xml!"

#def xmlEditor(xml):
    #TODO all this shit

