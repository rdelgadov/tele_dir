^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tele_dir
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2016-12-12)
------------------
* modify drone_config
* + Adds json dump content message to give more flexibility to the xml files.
  + Make execute with rosrun command.
  + Adds unit test to testing xmlhandler functions.
  + remake the xml drone and default configs.
  + Begin Pr2 xml config.
* XML Intern Editor
  + Adds xml intern editor with options to remove, modified and adds new topics, messages or buttons.
  + Refactor some functions like xmlCreator.
* gitignore
  + Remove from trackeds file the ".pyc" extension and the xml created.
* Publish Messages in different topics!
  + Adds xmlValidator the type messages validation
  + Adds read to message content
  - Remove velocity control. Pending resolve
* Merge branch 'master' of https://github.com/rdelgadov/tele_dir
* Refactoring:
  + Adds method signatures
  + Adds comments in the code
  + Modifies xml_validator to better reflect it's use. It now returns a boolean stating the correctness of the input XML file.
  + Removed various redundant exceptions that were unnecessary given the new structure of the method.
  + Alters many method and function names to better fit with the PEP 8 convention.
* Create README.md
* Refactoring beginnings:
  + Adds KeyBinding class
  + Segregates XmlHandler functions
  + Segregates RosFunctions functions
* Tele_dir version 1.0
  include xmlValidator, xmlCreator, selector to xml configs and navigation with this configuration.
* first commit
* Contributors: Rodrigo Alexis Delgado Vega, rdelgadov
