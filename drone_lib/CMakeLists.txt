cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 14)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

project (RealTimeLearning)

set(library_name droneLib)

find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )          

# CTello Shared Library =======================================================

add_library(${library_name} SHARED src/ctello.cpp)

add_library(droneLib::droneLib ALIAS ${library_name})

target_include_directories(${library_name} PRIVATE include)

target_link_libraries(${library_name} PRIVATE spdlog::spdlog ${Boost_LIBRARIES})

install(TARGETS ${library_name} DESTINATION lib)
install(FILES include/ctello.h DESTINATION include)

# CTello Command ==============================================================

add_executable(ctello-command src/ctello_command.cpp)

target_include_directories(ctello-command PRIVATE include)

target_link_libraries(ctello-command ${library_name})

install(TARGETS ctello-command DESTINATION bin)

# CTello State ================================================================

add_executable(ctello-state src/ctello_state.cpp)

target_include_directories(ctello-state PRIVATE include)

target_link_libraries(ctello-state ${library_name})

install(TARGETS ctello-state DESTINATION bin)

# CTello Stream ===============================================================

add_executable(ctello-stream src/ctello_stream.cpp)

target_include_directories(ctello-stream PRIVATE include)

target_link_libraries(ctello-stream ${library_name})
target_link_libraries(ctello-stream ${OpenCV_LIBS})

install(TARGETS ctello-stream DESTINATION bin)

# CTello Joystick =============================================================

#add_executable(ctello-joystick src/ctello_joystick.cpp)
#
#target_include_directories(ctello-joystick PRIVATE include)
#
#target_link_libraries(ctello-joystick ctello)
#
#install(TARGETS ctello-joystick DESTINATION bin)


#
## CTello Examples =============================================================
#
### Flip -----------------------------------------------------------------------
#add_executable(flip examples/flip.cpp)
#
#target_include_directories(flip PRIVATE include)
#
#target_link_libraries(flip ctello)
#target_link_libraries(flip ${OpenCV_LIBS})
#
### Follow ---------------------------------------------------------------------
#add_executable(follow examples/follow.cpp)
#
#target_include_directories(follow PRIVATE include)
#
#target_link_libraries(follow ctello)
#target_link_libraries(follow ${OpenCV_LIBS})
