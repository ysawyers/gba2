add_library(backend
    STATIC
    memory.cpp
    ppu.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    backend
)
