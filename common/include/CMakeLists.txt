# Top-Level CMakeList.txt

cmake_minimum_required (VERSION 2.6)
project ( FirstLab )

add_executable( FirstLab rendering.cpp)

find_package (SDL)

if ( NOT SDL_FOUND )
   message ( FATAL_ERROR "SDL not found!" )
endif ( NOT SDL_FOUND )

if(SDL_FOUND)
	include_directories(
		${SDL_INCLUDE_DIR}
		${PROJECT_SOURCE_DIR}/glm
	)
	#link_libraries(${SDL_LIBRARY})
	target_link_libraries(FirstLab ${SDL_LIBRARY})
endif(SDL_FOUND)

