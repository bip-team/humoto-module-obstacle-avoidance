if (HUMOTO_BUILD_TESTS)

    #HuMoTo demo 1
    #- qpOASES
    #- hrp2 (default)
    #- load parameters from yaml configuration files
    #- guess active set and solution
    #- log all data in Octave-parsable format to a given file (can be specified with a command line parameter)
    humoto_define_test(${HUMOTO_MODULE} "000" "qpOASES;config_yaml")

    #HuMoTo demo 2
    #- LexLS
    #- hrp2 (default)
    #- guess active set and solution
    #- log all data in Octave-parsable format to a given file (can be specified with a command line parameter)
    #humoto_define_test(${HUMOTO_MODULE} "003" "lexls")

    #Demo with HRP4 parameters, configurable optimization problem
    #- qpOASES
    #- hrp4
    #- guess active set and solution
    #- log all data in Octave-parsable format to a given file (can be specified with a command line parameter)
    #humoto_define_test(${HUMOTO_MODULE} "005" "qpOASES;config_yaml")

    #Performance test 1
    #- qpOASES / LexLS
    #- the number of simulation runs can be specified with a command line parameter (default = 20)
    #- time is measured with different hotstarting options
    #    a) no hotstarting
    #    b) guess solution
    #    c) guess active set
    #    d) guess both solution and active set
    #- measure time required for
    #    a) formulation of the preview horizon and optimization problem
    #    b) solution of the optimization problem
    #    c) the whole loop
    #- results are saved in Octave-parsable format
    #humoto_define_test(${HUMOTO_MODULE} "006" "" "lexls;qpOASES")

    #Demo with terminal constraint task in hierarchy
    #- qpOASES
    #- hrp2
    #- log all data in Octave-parsable format to a given file (can be specified with a command line parameter)
    #humoto_define_test(${HUMOTO_MODULE} "008" "qpOASES")

    # QuadProgpp demo
    #humoto_define_test(${HUMOTO_MODULE} "009" "QuadProgpp")
endif()
