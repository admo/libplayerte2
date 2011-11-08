# find pkg-config
find_package(PkgConfig REQUIRED)

# check for Player/Stage C++ libraries
pkg_check_modules(playercore REQUIRED playercore)

# includes
include_directories(${playercore_INCLUDE_DIRS})

add_library(playerte2 SHARED te.cpp)

target_link_libraries(playerte2 ${playercore_LIBRARIES})

set_target_properties (playerte2 PROPERTIES
    VERSION 0.0.1
    SOVERSION 0.0.1)

install (TARGETS playerte2 DESTINATION lib COMPONENT libraries)
install (FILES te.cfg DESTINATION share/player/config)
