message(STATUS "// =======================================")
message(STATUS "// Adding project test_chrono...")
message(STATUS "// =======================================")

cmake_policy(SET CMP0091 NEW)

project(test_chrono)

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
  COMPONENTS Irrlicht
  OPTIONAL_COMPONENTS PardisoMKL
  CONFIG)

if(NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

if(APPLE)
  set(CMAKE_MACOSX_BUNDLE ON)
endif()

include_directories(${CHRONO_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions("-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
  endif()
endif()

if(MSVC)
  add_definitions("-D_CRT_SECURE_NO_DEPRECATE") # avoids deprecation warnings
  add_definitions("-D_SCL_SECURE_NO_DEPRECATE") # avoids deprecation warnings
  add_definitions("-DNOMINMAX") # do not use MSVC's min/max macros
endif()

message(STATUS "Finding OPENMP...")
find_package(OpenMP REQUIRED)

message(STATUS "    OpenMP_CXX_FLAGS is ${OpenMP_CXX_FLAGS}.")

message(STATUS "Adding executables...")

set(DEMOS
  demo_MBS_collision_2d
  demo_MBS_collision_trimesh
  demo_MBS_collisionNSC
  demo_MBS_collisionSMC
  demo_MBS_convexhullChBodyEasySMC
  demo_MBS_convexhullSMC
  demo_MBS_fallingboxNSC
  demo_MBS_fallingboxSMC
  my_example
)

foreach(PROGRAM ${DEMOS})
  message(STATUS "    Adding executable: ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp)

  # --------------------------------------------------------------
  target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
  target_compile_options(${PROGRAM} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${PROGRAM} PUBLIC ${CHRONO_LINKER_FLAGS})

 
  target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES})

  add_DLL_copy_command()
endforeach(PROGRAM)