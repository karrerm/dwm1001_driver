cmake_minimum_required(VERSION 2.8.3)
project(dwm1001_driver)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

message("BinDir: ${CATKIN_PACKAGE_BIN_DESTINATION}")

cs_install_scripts(scripts/dwm1001_driver.py)

cs_export()

catkin_python_setup()
