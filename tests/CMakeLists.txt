add_executable(parse_tests parse_tests.cpp)

add_test(NAME parse_tests COMMAND parse_tests)

add_executable(robotics_scenario_tests robotics_scenario_tests.cpp)
add_test(NAME robotics_scenario_tests COMMAND robotics_scenario_tests)

add_executable(simulator_3d_tests simulator_3d_tests.cpp)
add_test(NAME simulator_3d_tests COMMAND simulator_3d_tests)
