#      Welcome to tele_dir ! !

The tele_dir package for ROS is a tool that allows for heavily customizable control support for most robots out there (it does _not_ support ActionLib). It is based around the idea of mapping keys to messages and topics in order to allow most robot interactions based on this paradigm. 

## Features

* A robust customization wizard and editor to allow a wide variety of different configuration schemes for your robot. 
* A validation system for custom made configuration files. 
* Ready-made configuration files for Parrot ArDrone 2.0 and most TurtleBots. 
* A non-greedy control system that allows for multiple control programs to work at the same time.


## How to use

Tele_dir is a console based program. It prompts for different inputs to comply with it's well-explained functions. The prompts start as soon as it is launched, asking if you wish to access the configuration wizard, the editor, or to launch the control support using the default configuration or a custom one.

The wizard will ask for the different topics and messages that will compose the key scheme, and will finally ask for keys to map to both the messages and topics previously input. The editor works similarly, but allowing you to edit stuff that was already there.

A lot of prompts allow for an exit writing "quit". 

## Installation

