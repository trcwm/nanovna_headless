cmake_minimum_required(VERSION 3.16)

project(hostapp VERSION 0.1.0)

add_executable(hostapp
    cobs.cpp
    main.cpp
)

add_executable(sincos_test
    sincos.cpp
)
