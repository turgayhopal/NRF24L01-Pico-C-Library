# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Pico/pico-sdk/tools/pioasm"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pioasm"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Turgay/OneDrive/Workspaces/personal-projects/NRF24L01-Library-Pico/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
