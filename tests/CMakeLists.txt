add_executable(parse_tests parse_tests.cpp)

add_test(NAME parse_tests COMMAND parse_tests)

add_executable(robotics_scenario_tests robotics_scenario_tests.cpp)
add_test(NAME robotics_scenario_tests COMMAND robotics_scenario_tests)
