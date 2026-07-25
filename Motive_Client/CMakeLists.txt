cmake_minimum_required(VERSION 3.20)

project(motive_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(
    NATNET_SDK_DIR
    "C:/Users/Ryan/Downloads/NatNet_SDK_4.5_windows/NatNetSDK"
)

add_executable(
    motive_client
    motive_client.cpp
)

target_include_directories(
    motive_client
    PRIVATE
    "${NATNET_SDK_DIR}/include"
)

target_link_libraries(
    motive_client
    PRIVATE
    "${NATNET_SDK_DIR}/lib/x64/NatNetLib.lib"
    ws2_32
)

add_custom_command(
    TARGET motive_client
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${NATNET_SDK_DIR}/lib/x64/NatNetLib.dll"
        "$<TARGET_FILE_DIR:motive_client>"
    COMMENT "Copying NatNetLib.dll beside motive_client.exe"
)