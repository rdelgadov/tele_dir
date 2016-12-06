import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Import modification to allow the import of XMLHandler
## TODO: refactor test names to fit convention
import unittest
import xml.etree.ElementTree as ET
import XmlHandler as xml

class XmlHandler_Testing(unittest.TestCase):
    def setUp(self):
        self.topic = ET.Element("topic", {'id': '1'} )
        name = ET.Element("name")
        msg_type = ET.Element("msg_type")
        name.text = "/cmd_vel"
        msg_type.text = "std_msgs/Empty"
        self.topic.insert(0,name)
        self.topic.insert(1,msg_type)

        self.buttons = ET.Element("buttons")
        button = ET.Element("button")
        key = ET.Element("key")
        key.text = 'F'
        b_message = ET.Element("message")
        b_message.text = '1'
        b_topic = ET.Element("topic")
        b_topic.text = '1'

        self.message = ET.Element("message", {'id': '1'})
        message_description = ET.Element("description")
        message_description.text = "Turn on"
        message_type = ET.Element("type")
        message_type.text = "std_msgs/Empty"
        message_content = ET.Element("content")
        message_content.text = ""
        self.message.insert(0, message_description)
        self.message.insert(1, message_type)
        self.message.insert(2, message_content)

        button.insert(0, key)
        button.insert(1, b_message)
        button.insert(2, b_topic)
        self.buttons.insert(0, button)

    def test_has_button(self):
        self.assertEqual(1, len(self.buttons.findall("button")))

    def test_delete_key_by_topic(self):
        list = xml.delete_key_by_topic(self.topic, self.buttons)
        self.assertEqual(1, len(list))
        self.assertEqual(0, len(self.buttons.findall("button")))

    def test_delete_key_by_message(self):
        list = xml.delete_key_by_message(self.message, self.buttons)
        self.assertEqual(1, len(list))
        self.assertEqual(0, len(self.buttons.findall("button")))

    def test_create_message(self):
        test_message = xml.createMessage(1, "Turn on", "std_msgs/Empty", "")
        self.assertEqual(self.message.attrib['id'], test_message.attrib['id'])
        self.assertEqual(self.message.find('description').text, test_message.find('description').text)
        self.assertEqual(self.message.find('type').text, test_message.find('type').text)
        self.assertEqual(self.message.find('content').text, test_message.find('content').text)

    def test_create_topic(self):
        test_topic = xml.createTopic(1, "/cmd_vel", "std_msgs/Empty")
        self.assertEqual(self.topic.attrib['id'], test_topic.attrib['id'])
        self.assertEqual(self.topic.find('name').text, test_topic.find('name').text)
        self.assertEqual(self.topic.find('msg_type').text, test_topic.find('msg_type').text)

    def test_validator(self):
        tree = ET.ElementTree()
        file = ET.Element("xml")
        tree._setroot(file)
        self.assertEquals(True ,xml.xml_validator("../Configs/asdf.xml"))
        self.assertEqual(False, xml.validator(tree))


if __name__ == '__main__':
    unittest.main()