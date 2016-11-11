import roslib.message
import sys
import xml.etree.ElementTree as ET

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


#if __name__ == '__main__':
 #   try:
  #      xmlvalidator("Configs/drone_config.xml")
   # except IOError as e:
    #    print e.args[1]
