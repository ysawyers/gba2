set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

find_package(SDL2 REQUIRED COMPONENTS SDL2)

add_executable(${PROJECT_NAME}
    main.cpp
)

add_subdirectory(libs)
add_subdirectory(backend)
add_subdirectory(local)

target_link_libraries(${PROJECT_NAME} PRIVATE backend)
target_link_libraries(${PROJECT_NAME} PRIVATE local)

target_include_directories(backend PUBLIC ${SOURCE_DIR})
target_include_directories(local PUBLIC ${SOURCE_DIR})
