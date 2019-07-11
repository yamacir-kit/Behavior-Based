cmake_minimum_required(VERSION 3.5.1) # ubuntu 16.04 default

set(MAJOR 0)
set(MINOR 0)

set(MAJOR_OFFSET 0)
set(MINOR_OFFSET 0)

execute_process(
  COMMAND git log --oneline
  COMMAND wc --lines
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE COMMITS
  )

math(EXPR SUBMINOR "${COMMITS} - ${MAJOR_OFFSET} - ${MINOR_OFFSET}")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/configure/package.xml.cmake)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configure/package.xml.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
endif()

execute_process(
  COMMAND grep -e "<name>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE PACKAGE_NAME)

execute_process(
  COMMAND grep -e "<version>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE PACKAGE_VERSION)

project(${PACKAGE_NAME} VERSION ${PACKAGE_VERSION} LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O2 -s")
set(CMAKE_CXX_STANDARD 14)

set(${PROJECT_NAME}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

execute_process(
  COMMAND grep -e "<depend>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE ${PROJECT_NAME}_PACKAGE_DEPENDS)

find_package(Boost  REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_PACKAGE_DEPENDS})

file(GLOB ${PROJECT_NAME}_MESSAGES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
if(${PROJECT_NAME}_MESSAGES)
  add_message_files(FILES ${${PROJECT_NAME}_MESSAGES})
endif()

file(GLOB ${PROJECT_NAME}_SERVICES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
if(${PROJECT_NAME}_SERVICES)
  add_service_files(FILES ${${PROJECT_NAME}_SERVICES})
endif()

file(GLOB ${PROJECT_NAME}_ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/action ${CMAKE_CURRENT_SOURCE_DIR}/action/*.action)
if(${PROEJECT_NAME}_ACTIONS)
  add_action_files(FILES ${${PROJECT_NAME}_ACTIONS})
endif()

if(${PROJECT_NAME}_PACKAGE_DEPENDS)
  foreach(EACH IN LISTS ${PROJECT_NAME}_PACKAGE_DEPENDS)
    if (EACH)
      string(REGEX MATCH "^(.*)_msgs$" REGEX_MATCHED ${EACH})
    endif()
    if (REGEX_MATCHED)
      list(APPEND ${PROJECT_NAME}_MESSAGE_DEPENDS ${REGEX_MATCHED})
    endif()
  endforeach(EACH)
endif()

if(${PROJECT_NAME}_MESSAGES OR ${PROJECT_NAME}_SERVICES OR ${PROJECT_NAME}_ACTIONS)
  generate_messages(DEPENDENCIES ${${PROJECT_NAME}_MESSAGE_DEPENDS})
endif()

if(EXISTS ${${PROJECT_NAME}_INCLUDE_DIR})
  catkin_package(
    INCLUDE_DIRS   ${${PROJECT_NAME}_INCLUDE_DIR}
    CATKIN_DEPENDS ${${PROJECT_NAME}_PACKAGE_DEPENDS})
endif()

string(TIMESTAMP ${PROJECT_NAME}_TIMESTAMP "%Y/%m/%d %H:%M:%S")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/configure/configure.hpp.cmake)
  # file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIR}/${PROJECT_NAME}/configure)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configure/configure.hpp.cmake
    ${${PROJECT_NAME}_INCLUDE_DIR}/${PROJECT_NAME}/configure.hpp)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/configure/README.md.cmake)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configure/README.md.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
endif()

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

link_directories(
  ${Boost_LIBRARY_DIRS}
  )

file(GLOB ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
if(${PROJECT_NAME}_SOURCES)
  foreach(EACH IN LISTS ${PROJECT_NAME}_SOURCES)
    string(REGEX REPLACE "/(.*/)*(.*).cpp" "\\2" TARGET ${EACH})
    add_executable(${TARGET} ${EACH})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${TARGET} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
  endforeach(EACH)
endif()

