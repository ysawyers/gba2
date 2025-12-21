set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(backend)
# add_subdirectory(common)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    backend
)
