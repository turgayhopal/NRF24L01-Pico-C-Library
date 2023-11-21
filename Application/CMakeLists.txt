add_executable(nrf24l01_library
        Src/main.c
        Inc/main.h)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(nrf24l01_library
        pico_binary_info
        hardware_spi
        hardware_gpio
        pico_stdlib
        library)

# enable usb output, disable uart output
pico_enable_stdio_usb(nrf24l01_library 1)
pico_enable_stdio_uart(nrf24l01_library 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(nrf24l01_library)