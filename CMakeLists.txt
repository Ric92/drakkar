##---------------------------------------------------------------------------------------------------------------------
##  DRAKKAR
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Ricardo Lopez Lopez (a.k.a. ric92) ricloplop@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.10 FATAL_ERROR)
project(drakkar VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

##################################################
######            Create project          ########
##################################################

file(GLOB_RECURSE DRAKKAR_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE DRAKKAR_HEADER_FILES "include/*.h" "include/*.inl")
add_library(${PROJECT_NAME} STATIC ${DRAKKAR_HEADER_FILES} ${DRAKKAR_SOURCE_FILES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME}
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")


#########################################
######			Thidparty 	       ######
#########################################

if (UNIX)
	find_package(Eigen3 REQUIRED)
endif()

find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
    target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_OPENCV)
    target_compile_definitions(${PROJECT_NAME} PUBLIC OPENCV)
endif()

find_package(realsense2 HINTS "/usr/local/lib/cmake/realsense2")
if(${realsense2_FOUND})
	MESSAGE(STATUS "Found librealsense v2, compiling RealSense devices wrapper")
	target_compile_definitions(${PROJECT_NAME} PUBLIC "ENABLE_LIBREALSENSE_V2")
	target_include_directories(${PROJECT_NAME} PUBLIC ${realsense2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${realsense2_LIBRARY})
endif()

find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
	
	target_compile_definitions(${PROJECT_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")
	# target_compile_options(${PROJECT_NAME} PUBLIC "-march=native -msse4.2 -mfpmath=sse -L/usr/lib/x86_64-linux-gnu")

	if(${PCL_VERSION} VERSION_GREATER 1.8)
		target_compile_definitions(${PROJECT_NAME} PUBLIC "HAS_PCL_1_8")
	else()
		target_compile_definitions(${PROJECT_NAME} PUBLIC "HAS_PCL_1_7")
	endif()

else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

#########################################
######			Tools    		   ######
#########################################

add_subdirectory(tools)

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS drakkar 
        EXPORT drakkarTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    drakkarConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )


# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS drakkar NAMESPACE drakkar:: FILE drakkarTargets.cmake)
if(${USE_SERVO})
  export(TARGETS drakkar-SCServo NAMESPACE drakkar:: FILE drakkarTargets.cmake)
endif()
# This generates drakkarTargets.cmake
install(EXPORT drakkarTargets
    FILE drakkarTargets.cmake
    NAMESPACE drakkar::
    DESTINATION lib/cmake/drakkar
    )

configure_file(drakkarConfig.cmake.in drakkarConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/drakkarConfig.cmake"
        DESTINATION lib/cmake/drakkar
        )


# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
