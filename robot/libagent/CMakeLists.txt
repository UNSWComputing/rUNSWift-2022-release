cmake_minimum_required(VERSION 2.8)
project(agent)
find_package(qibuild)
include_directories(..)
qi_create_lib(agent "libagent.cpp")
qi_use_lib(agent ALCOMMON)
