# CMake generated Testfile for 
# Source directory: /home/zhanghechao/workspace/ws/MOCHA/pmocha_experiments
# Build directory: /home/zhanghechao/workspace/ws/build/pmocha_experiments
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_grid_corridor_pipeline "/home/zhanghechao/workspace/ws/build/pmocha_experiments/test_grid_corridor_pipeline")
set_tests_properties(test_grid_corridor_pipeline PROPERTIES  _BACKTRACE_TRIPLES "/home/zhanghechao/workspace/ws/MOCHA/pmocha_experiments/CMakeLists.txt;112;add_test;/home/zhanghechao/workspace/ws/MOCHA/pmocha_experiments/CMakeLists.txt;0;")
add_test(test_mocha_single_corridor_pipeline "/home/zhanghechao/workspace/ws/build/pmocha_experiments/test_mocha_single_corridor_pipeline")
set_tests_properties(test_mocha_single_corridor_pipeline PROPERTIES  _BACKTRACE_TRIPLES "/home/zhanghechao/workspace/ws/MOCHA/pmocha_experiments/CMakeLists.txt;120;add_test;/home/zhanghechao/workspace/ws/MOCHA/pmocha_experiments/CMakeLists.txt;0;")
subdirs("mocha_planner")
