This is a demo of a humoto module for obstacle avoidance task in the MPC
hierarchy. The robot model used in ths module is hrp4.

Directory layout:
    humoto          -- a submodule which links the main repository of Humoto;
    extra_modules   -- a folder with extra modules for Humoto containing a
                       single mock example;
    Makefile        -- compilation script for the example module.

The example module is compiled with 'make all' command. You can learn the
necessary cmake parameters for Humoto compilation scripts by reading Makefile.

Tests are compiled with 'make tests'. The executable tests_000 gives output 
whcih can be read using  Python script included in the module which will plot
the trajectoryof the robot with obstacles.

You can use this repository as a template for new Humoto modules -- for this
purpose you have to replace the mock module 'extra_modules/example' with your
code.
