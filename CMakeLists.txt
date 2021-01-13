cmake_minimum_required(VERSION 3.0.2)
project(tm_driver_library)
add_compile_options(-std=c++14)

add_library(tm_driver_library SHARED
            src/tm_command.cpp
            src/tm_communication.cpp
            src/tm_driver.cpp
            src/tm_packet.cpp
            src/tm_print.cpp
            src/tm_robot_state.cpp
            src/tm_sct_communication.cpp
            src/tm_svr_communication.cpp
)

install(FILES libtm_driver_library.so DESTINATION lib/tm_driver)
install(DIRECTORY include/ DESTINATION lib/tm_driver/include)

