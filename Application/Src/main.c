//
// Created by Turgay on 9.11.2023.
//

#include "../Inc/main.h"

#include <tusb.h> // TinyUSB tud_cdc_connected()

int main(void) {

    // initialize all present standard stdio types
    stdio_init_all();

    // wait until the CDC ACM (serial port emulation) is connected
    while (!tud_cdc_connected()) {
        sleep_ms(10);
    }

    printf("USB Connected\r\n");

//#define RX_MODE_DEF
#define TX_MODE_DEF

    pin_control_t pin_control = {
            .sck = 2,
            .mosi = 3,
            .miso = 4,
            .csn = 5,
            .ce = 6,
    };

    spi_control_t spi_control = {
            .instance = spi0,
            .baudrate = 10 * 1000 * 1000
    };


    nrf_client_t nrf_client;
    nrf_resp_t nrf_resp;

    nrf_driver_create_client(&nrf_client);

    nrf_resp = nrf_client.io_configure(&pin_control, &spi_control);

    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("Configure IO OK \r\n");
    } else {
        printf("Configure IO Err \r\n");
    }

    nrf_control_t nrf_control = {
            .rf_channel = 0x01,
            .payload_size = 2,
            .rx_tx_config = PRIM_RX_PRX,
            .address_width = ADDR_WIDTH_5_BYTES,
            .crc_length_config = CRC_ENC_1_BYTE,
            .crc_state = CRC_ENABLE,
            .lna_gain_state = LNA_GAIN_ENABLE,
            .mask_max_rt_interrupt_state = MASK_MAX_RT_DISABLE,
            .mask_tx_ds_interrupt_state = MASK_TX_DS_DISABLE,
            .mask_rx_dr_interrupt_state = MASK_RX_DR_ENABLE,
            .power_state = POWER_UP,
            .retransmit_count = AUTO_RT_COUNT_DISABLE,
            .retransmit_delay = AUTO_RT_DELAY_250,
            .rf_data_rate = RF_DATA_RATE_2Mbps,
            .rf_pll_state = RF_PLL_LOCK_DISABLE,
            .rf_power = RF_POWER_0_dBm,
            .pipe_0_state = PIPE_0_ENABLE,
            .pipe_1_state = PIPE_1_ENABLE,
            .pipe_2_state = PIPE_2_ENABLE,
            .pipe_3_state = PIPE_3_ENABLE,
            .pipe_4_state = PIPE_4_ENABLE,
            .pipe_5_state = PIPE_5_ENABLE,
            .pipe_0_ack_state = AUTO_ACK_P0_ENABLE,
            .pipe_1_ack_state = AUTO_ACK_P1_ENABLE,
            .pipe_2_ack_state = AUTO_ACK_P2_ENABLE,
            .pipe_3_ack_state = AUTO_ACK_P3_ENABLE,
            .pipe_4_ack_state = AUTO_ACK_P4_ENABLE,
            .pipe_5_ack_state = AUTO_ACK_P5_ENABLE,
    };

#ifdef TX_MODE_DEF
    nrf_resp = nrf_client.initialise(NULL);
#endif

#ifdef RX_MODE_DEF
    nrf_resp = nrf_client.initialise(&nrf_control);
#endif

    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("Initialise OK \r\n");
    } else {
        printf("Initialise Err \r\n");
    }

    nrf_resp = nrf_client.rx_0_destination(NULL);
    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("RX 0 Dest Set OK \r\n");
    } else {
        printf("RX 0 Dest SetErr \r\n");
    }
    nrf_resp = nrf_client.rx_1_destination(NULL);
    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("RX 1 Dest Set OK \r\n");
    } else {
        printf("RX 1 Dest Set Err \r\n");
    }
    nrf_resp = nrf_client.tx_destination(NULL);
    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("TX Dest Set OK \r\n");
    } else {
        printf("TX Dest Set Err \r\n");
    }

    typedef struct rx_payload_s {
        uint8_t one;
        uint8_t two;
    } rx_payload_t;

    rx_payload_t payload;

    payload.one = 1;
    payload.two = 0;


#ifdef RX_MODE_DEF

    uint8_t pipe_number = 0;

    nrf_resp = nrf_client.start_listen();

    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("Start Listen OK \r\n");
    } else {
        printf("Start Listen Err \r\n");
    }

    while (1) {
        if (nrf_client.is_packet(&pipe_number, 10) == NRF_OK) {
            nrf_client.read_payload(&payload, sizeof(payload));
            printf("Is Package Ok \r\n");
        }

    }
#endif

#ifdef TX_MODE_DEF

    nrf_resp = nrf_client.ready_to_transmit();

    if (nrf_resp == NRF_CONFIGURE_OK) {
        printf("Start Transmit OK \r\n");
    } else {
        printf("Start Transmit Err \r\n");
    }

    while (1) {
        if (nrf_client.send_package(&payload, sizeof(payload)) == NRF_OK) {
            printf("Get ACK OK \r\n");
        }
    }

#endif

    return 0;

}