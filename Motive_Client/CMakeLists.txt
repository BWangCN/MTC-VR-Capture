cmake_minimum_required(VERSION 3.20)

project(motive_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(
    NATNET_SDK_DIR
    "C:/Users/Ryan/Downloads/NatNet_SDK_4.5_windows/NatNetSDK"
    CACHE PATH "NatNet SDK directory"
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

include(CTest)
if(BUILD_TESTING)
    add_executable(motive_client_tests tests/motive_client_tests.cpp)
    target_include_directories(motive_client_tests PRIVATE "${NATNET_SDK_DIR}/include")
    target_link_libraries(motive_client_tests PRIVATE "${NATNET_SDK_DIR}/lib/x64/NatNetLib.lib" ws2_32)
    add_custom_command(TARGET motive_client_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${NATNET_SDK_DIR}/lib/x64/NatNetLib.dll" "$<TARGET_FILE_DIR:motive_client_tests>")
    add_test(NAME motive_client_callback_checks COMMAND motive_client_tests)
    add_executable(mocap_recording_tests tests/recording_tests.cpp)
    target_include_directories(mocap_recording_tests PRIVATE "${NATNET_SDK_DIR}/include")
    target_link_libraries(mocap_recording_tests PRIVATE "${NATNET_SDK_DIR}/lib/x64/NatNetLib.lib" ws2_32)
    add_custom_command(TARGET mocap_recording_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${NATNET_SDK_DIR}/lib/x64/NatNetLib.dll" "$<TARGET_FILE_DIR:mocap_recording_tests>")
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME mocap_recording_checks COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_recording_files.py" "$<TARGET_FILE:mocap_recording_tests>")
    else()
        add_test(NAME mocap_recording_checks COMMAND mocap_recording_tests "${CMAKE_CURRENT_BINARY_DIR}/recording-checks")
    endif()
endif()
