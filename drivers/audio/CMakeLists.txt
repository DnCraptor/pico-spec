add_library(audio INTERFACE)

target_sources(audio INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/audio.c
		${CMAKE_CURRENT_LIST_DIR}/audio.h
		${CMAKE_CURRENT_LIST_DIR}/pcm5122_init.c
		${CMAKE_CURRENT_LIST_DIR}/pcm5122_init.h
)

target_link_libraries(audio INTERFACE hardware_pio hardware_clocks hardware_i2c)

target_include_directories(audio INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}
)

pico_generate_pio_header(audio
  ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio
)
