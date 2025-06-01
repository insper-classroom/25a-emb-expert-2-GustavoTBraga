add_executable(pico_emb
        hid_buttons.c
        main.c
)

set_target_properties(pico_emb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Link the Project to extra libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  pico_stdlib
  pico_btstack_classic
  pico_btstack_cyw43
  pico_cyw43_arch_none
)

# Including header files directly from project directory
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(pico_emb)
