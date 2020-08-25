INCLUDE_DIRECTORIES(.)
add_definitions(-std=c++11)

aux_source_directory(./ FRAMEWORK_SRCS)
FIND_PACKAGE(HPlugin REQUIRED)

CONFIGURE_FILE(hirop_perception.cmake.in ${PROJECT_BINARY_DIR}/hirop_perceptionConfig.cmake @ONLY)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/perception ${tf_INCLUDE_DIRS})

ADD_LIBRARY(hirop_perception SHARED ${FRAMEWORK_SRCS})
TARGET_LINK_LIBRARIES(hirop_perception pthread  ${HPlugin_LIBRARIES} ${YAML_CPP_LIBRARIES})

## 安装目标文件
INSTALL(TARGETS hirop_perception
        EXPORT hirop_perception
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)

## 安装头文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/hirop)

## 安装cmake配置文件
INSTALL(FILES "${PROJECT_BINARY_DIR}/hirop_perceptionConfig.cmake" DESTINATION lib/cmake/hirop_perception/)

#add_executable(hirop_perecption_refilters_test ${FRAMEWORK_SRCS})

#TARGET_LINK_LIBRARIES(hirop_perecption_refilters_test ${PCL_LIBRARIES}  pthread log4cpp ${HPlugin_LIBRARIES} ${YAML_CPP_LIBRARIES})
