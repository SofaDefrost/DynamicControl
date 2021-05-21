[![Information](https://img.shields.io/badge/Team-_website-blue.svg)](https://team.inria.fr/defrost/)
[![Information](https://img.shields.io/badge/SoftRobots-_website-purple.svg)](https://project.inria.fr/softrobot/)
[![Contact](https://img.shields.io/badge/contact-form-green.svg)](https://project.inria.fr/softrobot/contact/) 

# DYNAMIC CONTROL DIRECTORY

Matlab toolbox to model soft robots and design dynamic controllers.
It relies on the modeling algorithms implemented within the [SoftRobots plugin](https://github.com/SofaDefrost/SoftRobots).

Usage
=====

- launch *SoftRobot_app*
    - set environment variable *env_var=startup()*
    - launch app *SoftRobot_app(env_var)*


Installation
============

PYTHON
------

- Install python with matplotlib and tinker lib


SOFA
----

- Install SOFA and the Soft Robot plugin

- See https://project.inria.fr/softrobot/install-get-started-2/
 
- Add runSofa to path

Model Order Reduction
---------------------

This plugin for SOFA is optional, it is however required to compute reduced order models.

- Install MOR plugin https://modelorderreduction.readthedocs.io/en/latest/usage/install/install.html

- with required all dependencies 
    - https://modelorderreduction.readthedocs.io/en/latest/usage/install/requirement.html
    - (see Package dependencies / Install all )
    - the model reduction plugin requires sofa-launcher:
        - add this line to your .bashrc file:
        - export PYTHONPATH=/PathToYourSofaSrcFolder/tools/sofa-launcher
    - enable saveVelocitySnapshots :
        - uncomment two last lines of file ModelOrderReduction/python/mor/reduction/template/phase1_snapshots.py


Matlab:
-------

- update file "startup.m" according to your files architecture
- the following may be optional, but required to design a controller
    - yalmip: add yalmip files to path, and add solvers (mosek, sedumi ...)
    - mosek solver
        - add /mosek/8/tools and /mosek/toolbox/ to path
        - add /home/username/mosek/mosek.lic to path
    - install the [MORE toolbox](https://w3.onera.fr/more/)
- launch *tests_RunAndReport.m* to check that the installation was successfull


Documentation 
-------------

- type help FILENAME (or DIRNAME) or doc FILENAME (DIRNAME)


Use this toolbox without Sofa installed
---------------------------------------

This app can be used without Sofa installed if the required data are provided.
Using the 'Build new model' callback will propose two options:
- if required text files (system matrices, step time etc...) 
      are present in the directory 'SofaFolder.data' (this is defined in 
      file 'startup.m'.), then the app can build a model based on these text files.
- if a directory 'savedModels' exists and contains a '.mat'
      corresponding to the robot studied, the app can load this model.