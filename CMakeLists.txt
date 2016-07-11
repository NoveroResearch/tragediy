cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0025 NEW)
project(tragediy)
set(tragediy_VERSION_MAJOR 0)
set(tragediy_VERSION_MINOR 1)

# --- function to create source_groups with folders ---

function(create_source_group sourceGroupName relativeSourcePath)
	FOREACH(currentSourceFile ${ARGN})
		FILE(RELATIVE_PATH folder ${relativeSourcePath} ${currentSourceFile})
		get_filename_component(filename ${folder} NAME)
		string(REPLACE ${filename} "" folder ${folder})
		if(NOT folder STREQUAL "")
			string(REGEX REPLACE "/+$" "" folderlast ${folder})
			string(REPLACE "/" "\\" folderlast ${folderlast})
			SOURCE_GROUP("${sourceGroupName}\\${folderlast}" FILES ${currentSourceFile})
		else()
			SOURCE_GROUP("${sourceGroupName}" FILES ${currentSourceFile})
		endif(NOT folder STREQUAL "")
	ENDFOREACH(currentSourceFile ${ARGN})
endfunction(create_source_group)

# --- Check Mandatory Built-time Dependencies ---

# Guide FindBoost.cmake's selection of boost libraries
#set( Boost_USE_STATIC_LIBS OFF CACHE BOOL "Force static linking of Boost libraries." )

# Boost threads is only available as a multithreaded version so force FindBoost.cmake to use the multithreaded versions.
#set( Boost_USE_MULTITHREADED ON )

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options system filesystem REQUIRED)
if(NOT Boost_FOUND)
	message(error ": Boost library not found.")
endif()
include_directories(system ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
link_libraries(${Boost_LIBRARIES})

# Disable Boost auto-linking since it conflicts with cmake:
ADD_DEFINITIONS    ( -DBOOST_ALL_NO_LIB    )

# Ensure C++14 compatibility.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
		message(FATAL_ERROR "The project requires your compiler to support C++14 language features. The GCC only supports the relevant features as off version 5.0.")
	endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.4")
		message(FATAL_ERROR "The project requires your compiler to support C++14 language features. The clang compiler only supports the relevant features as off version 3.4.")
	endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(FATAL_ERROR "The project requires your compiler to support C++14 language features. Apple's clang compiler only supports the relevant features as off version 5.1.")
	endif()
endif()

# --- Check Optional Built-time Dependencies ---

if(USE_OPTIONALDEP)
	find_package(OptionalDep REQUIRED)

	set(HAVE_OPTIONALDEP BOOL ON)
endif()


# --- Set Compiler Switches ---

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wall -Werror -pedantic -g")
endif()

# --- Collect Sources and Headers ---

set(TRAGEDIY_SOURCES
	${PROJECT_SOURCE_DIR}/src/track/AnkiMap.cpp
	${PROJECT_SOURCE_DIR}/src/track/Lane.cpp
	${PROJECT_SOURCE_DIR}/src/track/LaneArcTile.cpp
	${PROJECT_SOURCE_DIR}/src/track/LaneLineTile.cpp
	${PROJECT_SOURCE_DIR}/src/track/LaneTileBase.cpp
	${PROJECT_SOURCE_DIR}/src/track/LocationTable.cpp
	${PROJECT_SOURCE_DIR}/src/track/Track.cpp
	${PROJECT_SOURCE_DIR}/src/util/AssertionError.cpp
	${PROJECT_SOURCE_DIR}/src/tragediy.cpp
)

file(GLOB_RECURSE TRAGEDIY_HEADERS ${PROJECT_SOURCE_DIR}/inc/tragediy/*.h)

#set(TRAGEDIY_SOURCES_ALL ${ANKITED_SOURCES})
#set(TRAGEDIY_HEADERS_ALL ${ANKITED_HEADERS})

if( MSVC )
   source_group("inc\\tragediy"        FILES ${TRAGEDIY_HEADERS})
   source_group("src"                  FILES ${TRAGEDIY_SOURCES})
endif()

if(CMAKE_GENERATOR STREQUAL Xcode)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedef")

    list(SORT TRAGEDIY_HEADERS)
    list(SORT TRAGEDIY_SOURCES)

    create_source_group("src" ${PROJECT_SOURCE_DIR}/src ${TRAGEDIY_SOURCES})
    create_source_group("inc" ${PROJECT_SOURCE_DIR}/inc ${TRAGEDIY_HEADERS})

    set(TRAGEDIY_SOURCES ${TRAGEDIY_SOURCES} ${TRAGEDIY_HEADERS})
endif()

# --- Set Include Directories ---

include_directories("${PROJECT_BINARY_DIR}/inc")
include_directories("${PROJECT_SOURCE_DIR}/inc")


# --- Create Targets ---

add_executable(tragediy ${TRAGEDIY_SOURCES})
#target_link_libraries(tragediy ${EXTRA_LIBS})

install(TARGETS tragediy DESTINATION bin)
