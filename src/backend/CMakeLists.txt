add_library(backend
    STATIC
    cpu.cpp
)

# target_link_libraries(backend
#     PRIVATE
#     common
# )

target_include_directories(backend
    PUBLIC
    ${SOURCE_DIR}
)