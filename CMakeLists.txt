cmake_minimum_required(VERSION 3.5.1) # ubuntu 16.04 default

# ==============================================================================
#   Calculate Project Version
# ==============================================================================
set(MAJOR 0)
set(MAJOR_OFFSET 0)

set(MINOR 0)
set(MINOR_OFFSET 0)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
  execute_process(
    COMMAND git rev-list --all --count
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE REVISION_COUNT
    )
  math(EXPR PATCH "${REVISION_COUNT} - ${MAJOR_OFFSET} - ${MINOR_OFFSET}")
else()
  set(PATCH)
endif()

# ==============================================================================
#   Configure Package.xml
# ==============================================================================
set(${PROJECT_NAME}_CONFIGURE ${CMAKE_CURRENT_SOURCE_DIR}/configure)

if(EXISTS ${${PROJECT_NAME}_CONFIGURE}/package.xml.cmake)
  configure_file(
    ${${PROJECT_NAME}_CONFIGURE}/package.xml.cmake
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

execute_process(
  COMMAND grep -e "<depend>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE PACKAGE_DEPENDS)

# ==============================================================================
#   Project Settings
# ==============================================================================
project(${PACKAGE_NAME} VERSION ${PACKAGE_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Og")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -mtune=native -march=native -DNDEBUG")
set(CMAKE_CXX_STANDARD 14)

find_package(Boost  REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDS})

# ==============================================================================
#   Generate Messages, Services and Actions (untested)
# ==============================================================================
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

if(PACKAGE_DEPENDS)
  foreach(EACH IN LISTS PACKAGE_DEPENDS)
    if(EACH)
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

# ==============================================================================
#   Configure Header
# ==============================================================================
set(${PROJECT_NAME}_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(EXISTS ${${PROJECT_NAME}_INCLUDE})
  catkin_package(
    INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE}
    CATKIN_DEPENDS ${PACKAGE_DEPENDS})
endif()

if(EXISTS ${${PROJECT_NAME}_CONFIGURE}/configure.hpp.cmake)
  string(TIMESTAMP ${PROJECT_NAME}_TIMESTAMP "%Y/%m/%d %H:%M:%S")
  configure_file(
    ${${PROJECT_NAME}_CONFIGURE}/configure.hpp.cmake
    ${${PROJECT_NAME}_INCLUDE}/${PROJECT_NAME}/configure.hpp)
endif()

# ==============================================================================
#   Configure README
# ==============================================================================
if(EXISTS ${${PROJECT_NAME}_CONFIGURE}/README.md.cmake)
  configure_file(
    ${${PROJECT_NAME}_CONFIGURE}/README.md.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
endif()

# ==============================================================================
#   Configure Doxyfile
# ==============================================================================
set(${PROJECT_NAME}_CONFIGURE ${CMAKE_CURRENT_SOURCE_DIR}/configure)
if(EXISTS ${${PROJECT_NAME}_CONFIGURE}/doxyfile.cmake)
  configure_file(
    ${${PROJECT_NAME}_CONFIGURE}/doxyfile.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/doxyfile)
endif()

# ==============================================================================
#   Main
# ==============================================================================
include_directories(
  ${${PROJECT_NAME}_INCLUDE}
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
    list(APPEND ${PROJECT_NAME}_TARGETS ${TARGET})
    add_executable(${TARGET} ${EACH})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${TARGET} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
  endforeach(EACH)
endif()

install(
  TARGETS ${${PROJECT_NAME}_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

