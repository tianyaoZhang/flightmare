# CMake generated Testfile for 
# Source directory: /home/tyZhang/Documents/AgileFlight/flightmare/flightlib
# Build directory: /home/tyZhang/Documents/AgileFlight/flightmare/flightlib/build/temp.linux-x86_64-3.8
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_lib "test_lib")
set_tests_properties(test_lib PROPERTIES  _BACKTRACE_TRIPLES "/home/tyZhang/Documents/AgileFlight/flightmare/flightlib/CMakeLists.txt;280;add_test;/home/tyZhang/Documents/AgileFlight/flightmare/flightlib/CMakeLists.txt;0;")
add_test(test_unity_bridge "test_unity_bridge")
set_tests_properties(test_unity_bridge PROPERTIES  _BACKTRACE_TRIPLES "/home/tyZhang/Documents/AgileFlight/flightmare/flightlib/CMakeLists.txt;290;add_test;/home/tyZhang/Documents/AgileFlight/flightmare/flightlib/CMakeLists.txt;0;")
subdirs("externals/pybind11-src")
subdirs("../../externals/googletest-build")
