import os
import std_msgs.msg
import unittest
import AuxFuns as aux

from geometry_msgs.msg import Twist

## Aquí se testean las funciones auxiliares.

class AuxFuns_Test(unittest.TestCase):
    def setUp(self):
        message = {'a':{'x':10,'y':5,'z':2},'b':{'x':10,'y':5,'z':2}}




    def message_param_editor_test(self):
        output = aux.message_param_editor(message, lambda: 20, lambda: 50, lambda: 100, lambda: 20, lambda: 50, lambda: 100)
        print output




