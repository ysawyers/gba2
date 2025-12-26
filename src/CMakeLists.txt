set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME}
    main.cpp
)

add_subdirectory(libs)
add_subdirectory(backend)
add_subdirectory(local)

target_include_directories(backend
    PUBLIC
    ${SOURCE_DIR}
)
