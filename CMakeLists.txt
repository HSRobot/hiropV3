cmake_minimum_required (VERSION 2.6)
project (HiropFramework)

FILE(GLOB_RECURSE LibFiles "include/*.h")

add_custom_target(headers SOURCES ${LibFiles})

INCLUDE_DIRECTORIES(include)

## 声明当前处于Linux平台环境
add_definitions(-D__LINUX__)

add_subdirectory(src)

add_subdirectory(test)
