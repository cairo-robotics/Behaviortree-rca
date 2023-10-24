# CMake generated Testfile for 
# Source directory: /home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner
# Build directory: /home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/build/src/jsontestrunner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(jsoncpp_readerwriter "/usr/bin/python3.8" "-B" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/build/bin/jsontestrunner_exe" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter PROPERTIES  WORKING_DIRECTORY "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/data" _BACKTRACE_TRIPLES "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/CMakeLists.txt;43;add_test;/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/CMakeLists.txt;0;")
add_test(jsoncpp_readerwriter_json_checker "/usr/bin/python3.8" "-B" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "--with-json-checker" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/build/bin/jsontestrunner_exe" "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter_json_checker PROPERTIES  WORKING_DIRECTORY "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/../../test/data" _BACKTRACE_TRIPLES "/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/CMakeLists.txt;47;add_test;/home/dt/HRIPapers/BehaviorTreeWork/src/jsoncpp/src/jsontestrunner/CMakeLists.txt;0;")
