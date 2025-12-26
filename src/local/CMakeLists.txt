add_library(local
    STATIC
    window.cpp
)

target_link_libraries(local
    PRIVATE
    imgui
)
