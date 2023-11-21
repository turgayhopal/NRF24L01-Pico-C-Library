add_library(library

        Inc/nrf24l01.h Src/nrf24l01.c)

target_include_directories(library PUBLIC
        Src
        Inc
        )

target_link_libraries(library
        hardware_spi
        hardware_gpio
        pico_stdlib)