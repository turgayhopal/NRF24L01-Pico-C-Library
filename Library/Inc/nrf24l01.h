//
// Created by Turgay on 9.11.2023.
//

#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

typedef enum nrf_reset_reg_val_e {
    STATUS_RESET = 0x70
} nrf_reset_reg_val_t;

typedef enum nrf_irq_check_e {
    RX_DR = 0x40,
    TX_DS = 0x20,
    MAX_RT = 0x10,
    TX_FULL = 0x01
} nrf_irq_check_t;

typedef enum nrf_resp_e {
    NRF_ERR = 0x08,
    NRF_OK = 0x04,
    NRF_CONFIGURE_OK = 0x02,
    NRF_CONFIGURE_ERR = 0x01,
} nrf_resp_t;

typedef enum register_commands_e {
    R_REGISTER = 0x1F,
    W_REGISTER = 0x20,
    NOP = 0xFF
} register_commands_t;

typedef enum fifo_commands_e {
    FLUSH_TX = 0xE1,
    FLUSH_RX = 0xE2,
} fifo_commands_t;


typedef enum payload_commands_e {
    R_RX_PAYLOAD = 0x61,
    W_TX_PAYLOAD = 0xA0,
    REUSE_TX_PL = 0xE3
} payload_commands_t;

typedef enum register_map_e {
    CONFIG = 0x00,
    EN_AA = 0x01,
    EN_RX_ADDR = 0x02,
    SETUP_AW = 0x03,
    SETUP_RE_TR = 0x04,
    RF_CH = 0x05,
    RF_SETUP = 0x06,
    STATUS = 0x07,
    OBSERVE_TX = 0x08,
    CD = 0x09,
    RX_ADDR_P0 = 0x0A,
    RX_ADDR_P1 = 0x0B,
    RX_ADDR_P2 = 0x0C,
    RX_ADDR_P3 = 0x0D,
    RX_ADDR_P4 = 0x0E,
    RX_ADDR_P5 = 0x0F,
    TX_ADDR = 0x10,
    RX_PW_P0 = 0x11,
    RX_PW_P1 = 0x12,
    RX_PW_P2 = 0x13,
    RX_PW_P3 = 0x14,
    RX_PW_P4 = 0x15,
    RX_PW_P5 = 0x16,
    FIFO_STATUS = 0x17,
} register_map_t;

typedef enum prim_rx_params_e {
    PRIM_RX_PTX = 0x00,
    PRIM_RX_PRX = 0x01,
} prim_rx_params_t;

typedef enum power_params_e {
    POWER_UP = 0x02,
    POWER_DOWN = 0x00,
} power_params_t;

typedef enum crc_len_params_e {
    CRC_ENC_1_BYTE = 0x00,
    CRC_ENC_2_BYTE = 0x04,
} crc_len_params_t;

typedef enum crc_en_params_e {
    CRC_DISABLE = 0x00,
    CRC_ENABLE = 0x08,
} crc_en_params_t;

typedef enum mask_max_rt_params_e {
    MASK_MAX_RT_DISABLE = 0x00,
    MASK_MAX_RT_ENABLE = 0x10,
} mask_max_rt_params_t;

typedef enum mask_max_tx_params_e {
    MASK_TX_DS_DISABLE = 0x00,
    MASK_TX_DS_ENABLE = 0x20,
} mask_tx_ds_params_t;

typedef enum mask_rx_dr_params_e {
    MASK_RX_DR_DISABLE = 0x00,
    MASK_RX_DR_ENABLE = 0x40
} mask_rx_dr_params_t;

typedef enum en_aa_pipe_0_params_e {
    AUTO_ACK_P0_ENABLE = 0x01,
    AUTO_ACK_P0_DISABLE = 0x00,
} en_aa_pipe_0_params_t;

typedef enum en_aa_pipe_1_params_e {
    AUTO_ACK_P1_ENABLE = 0x02,
    AUTO_ACK_P1_DISABLE = 0x00,
} en_aa_pipe_1_params_t;

typedef enum en_aa_pipe_2_params_e {
    AUTO_ACK_P2_ENABLE = 0x04,
    AUTO_ACK_P2_DISABLE = 0x00,
} en_aa_pipe_2_params_t;

typedef enum en_aa_pipe_3_params_e {
    AUTO_ACK_P3_ENABLE = 0x08,
    AUTO_ACK_P3_DISABLE = 0x00,
} en_aa_pipe_3_params_t;

typedef enum en_aa_pipe_4_params_e {
    AUTO_ACK_P4_ENABLE = 0x10,
    AUTO_ACK_P4_DISABLE = 0x00,
} en_aa_pipe_4_params_t;

typedef enum en_aa_pipe_5_params_e {
    AUTO_ACK_P5_ENABLE = 0x20,
    AUTO_ACK_P5_DISABLE = 0x00
} en_aa_pipe_5_params_t;

typedef enum en_rx_pipe_0_params_e {
    PIPE_0_ENABLE = 0x01,
    PIPE_0_DISABLE = 0x00,
} en_rx_pipe_0_params_t;

typedef enum en_rx_pipe_1_params_e {
    PIPE_1_ENABLE = 0x02,
    PIPE_1_DISABLE = 0x00,
} en_rx_pipe_1_params_t;

typedef enum en_rx_pipe_2_params_e {
    PIPE_2_ENABLE = 0x04,
    PIPE_2_DISABLE = 0x00,
} en_rx_pipe_2_params_t;

typedef enum en_rx_pipe_3_params_e {
    PIPE_3_ENABLE = 0x08,
    PIPE_3_DISABLE = 0x00,
} en_rx_pipe_3_params_t;

typedef enum en_rx_pipe_4_params_e {
    PIPE_4_ENABLE = 0x10,
    PIPE_4_DISABLE = 0x00,
} en_rx_pipe_4_params_t;

typedef enum en_rx_pipe_5_params_e {
    PIPE_5_ENABLE = 0x20,
    PIPE_5_DISABLE = 0x00
} en_rx_pipe_5_params_t;

typedef enum aw_params_e {
    ADDR_WIDTH_ILLEGAL = 0x00,
    ADDR_WIDTH_3_BYTES = 0x01,
    ADDR_WIDTH_4_BYTES = 0x02,
    ADDR_WIDTH_5_BYTES = 0x03
} aw_params_t;

typedef enum re_tr_delay_params_e {
    AUTO_RT_DELAY_250 = 0x00,
    AUTO_RT_DELAY_500 = 0x10,
    AUTO_RT_DELAY_750 = 0x20,
    AUTO_RT_DELAY_1000 = 0x30,
    AUTO_RT_DELAY_1250 = 0x40,
    AUTO_RT_DELAY_1500 = 0x50,
    AUTO_RT_DELAY_1750 = 0x60,
    AUTO_RT_DELAY_2000 = 0x70,
    AUTO_RT_DELAY_2250 = 0x80,
    AUTO_RT_DELAY_2500 = 0x90,
    AUTO_RT_DELAY_2750 = 0xA0,
    AUTO_RT_DELAY_3000 = 0xB0,
    AUTO_RT_DELAY_4250 = 0xC0,
    AUTO_RT_DELAY_4500 = 0xD0,
    AUTO_RT_DELAY_4750 = 0xE0,
    AUTO_RT_DELAY_4000 = 0xF0,
} re_tr_delay_params_t;

typedef enum re_tr_count_params_e {
    AUTO_RT_COUNT_DISABLE = 0x00,
    AUTO_RT_COUNT_1 = 0x01,
    AUTO_RT_COUNT_2 = 0x02,
    AUTO_RT_COUNT_3 = 0x03,
    AUTO_RT_COUNT_4 = 0x04,
    AUTO_RT_COUNT_5 = 0x05,
    AUTO_RT_COUNT_6 = 0x06,
    AUTO_RT_COUNT_7 = 0x07,
    AUTO_RT_COUNT_8 = 0x08,
    AUTO_RT_COUNT_9 = 0x09,
    AUTO_RT_COUNT_10 = 0x0A,
    AUTO_RT_COUNT_11 = 0x0B,
    AUTO_RT_COUNT_12 = 0x0C,
    AUTO_RT_COUNT_13 = 0x0D,
    AUTO_RT_COUNT_14 = 0x0E,
    AUTO_RT_COUNT_15 = 0x0F
} re_tr_count_params_t;

typedef enum rf_gain_params_e {
    LNA_GAIN_DISABLE = 0x00,
    LNA_GAIN_ENABLE = 0x01,
} rf_gain_params_t;

typedef enum rf_power_params_e {
    RF_POWER_18_dBm = 0x00,
    RF_POWER_12_dBm = 0x02,
    RF_POWER_6_dBm = 0x04,
    RF_POWER_0_dBm = 0x06,
} rf_power_params_t;

typedef enum rf_data_rate_params_e {
    RF_DATA_RATE_1Mbps = 0x00,
    RF_DATA_RATE_2Mbps = 0x08,
} rf_data_rate_params_t;

typedef enum rf_pll_params_e {
    RF_PLL_LOCK_DISABLE = 0x00,
    RF_PLL_LOCK_ENABLE = 0x10
} rf_pll_params_t;

typedef struct pipe_0_addr_e {
    uint8_t addr_0;
    uint8_t addr_1;
    uint8_t addr_2;
    uint8_t addr_3;
    uint8_t addr_4;
} pipe_0_addr_t;

typedef struct pipe_1_addr_e {
    uint8_t addr_0;
    uint8_t addr_1;
    uint8_t addr_2;
    uint8_t addr_3;
    uint8_t addr_4;
} pipe_1_addr_t;

typedef struct pipe_2_addr_e {
    uint8_t addr_0;
} pipe_2_addr_t;

typedef struct pipe_3_addr_e {
    uint8_t addr_0;
} pipe_3_addr_t;

typedef struct pipe_4_addr_e {
    uint8_t addr_0;
} pipe_4_addr_t;

typedef struct pipe_5_addr_e {
    uint8_t addr_0;
} pipe_5_addr_t;

typedef struct tx_addr_e {
    uint8_t addr_0;
    uint8_t addr_1;
    uint8_t addr_2;
    uint8_t addr_3;
    uint8_t addr_4;
} tx_addr_t;

typedef struct pin_control_s {
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
    uint8_t csn;
    uint8_t ce;
    uint8_t irq;
} pin_control_t;

typedef struct spi_control_s {
    spi_inst_t *instance;
    uint32_t baudrate;
} spi_control_t;

typedef enum nrf_device_mode_e {
    POWER_DOWN_MODE,
    STANDBY_I,
    STANDBY_II,
    TX_MODE,
    RX_MODE
} nrf_device_mode_t;

typedef struct nrf_control_s {
    prim_rx_params_t rx_tx_config;
    power_params_t power_state;
    uint8_t rf_channel;
    uint8_t payload_size;
    crc_len_params_t crc_length_config;
    crc_en_params_t crc_state;
    mask_max_rt_params_t mask_max_rt_interrupt_state;
    mask_tx_ds_params_t mask_tx_ds_interrupt_state;
    mask_rx_dr_params_t mask_rx_dr_interrupt_state;
    en_aa_pipe_0_params_t pipe_0_ack_state;
    en_aa_pipe_1_params_t pipe_1_ack_state;
    en_aa_pipe_2_params_t pipe_2_ack_state;
    en_aa_pipe_3_params_t pipe_3_ack_state;
    en_aa_pipe_4_params_t pipe_4_ack_state;
    en_aa_pipe_5_params_t pipe_5_ack_state;
    en_rx_pipe_0_params_t pipe_0_state;
    en_rx_pipe_1_params_t pipe_1_state;
    en_rx_pipe_2_params_t pipe_2_state;
    en_rx_pipe_3_params_t pipe_3_state;
    en_rx_pipe_4_params_t pipe_4_state;
    en_rx_pipe_5_params_t pipe_5_state;
    aw_params_t address_width;
    re_tr_delay_params_t retransmit_delay;
    re_tr_count_params_t retransmit_count;
    rf_gain_params_t lna_gain_state;
    rf_power_params_t rf_power;
    rf_data_rate_params_t rf_data_rate;
    rf_pll_params_t rf_pll_state;
} nrf_control_t;

typedef struct nrf_driver_s {
    pin_control_t pin_control;
    spi_control_t spi_control;
    nrf_control_t nrf_control;
    nrf_device_mode_t device_mode;
    tx_addr_t tx_addr;
    pipe_0_addr_t pipe_0_addr;
    pipe_1_addr_t pipe_1_addr;
    pipe_2_addr_t pipe_2_addr;
    pipe_3_addr_t pipe_3_addr;
    pipe_4_addr_t pipe_4_addr;
    pipe_5_addr_t pipe_5_addr;
} nrf_driver_t;

typedef struct nrf_client_s {
    nrf_resp_t (*io_configure)(pin_control_t *pin_control, spi_control_t *spi_control);

    nrf_resp_t (*initialise)(nrf_control_t *nrf_control);

    nrf_resp_t (*rx_0_destination)(pipe_0_addr_t *pipe_0_addr);

    nrf_resp_t (*rx_1_destination)(pipe_1_addr_t *pipe_1_addr);

    nrf_resp_t (*rx_2_destination)(pipe_2_addr_t *pipe_2_addr);

    nrf_resp_t (*rx_3_destination)(pipe_3_addr_t *pipe_3_addr);

    nrf_resp_t (*rx_4_destination)(pipe_4_addr_t *pipe_4_addr);

    nrf_resp_t (*rx_5_destination)(pipe_5_addr_t *pipe_5_addr);

    nrf_resp_t (*tx_destination)(tx_addr_t *tx_addr);

    nrf_resp_t (*payload_size)(size_t size);

    nrf_resp_t (*auto_retransmission)(re_tr_delay_params_t delay, re_tr_count_params_t count);

    nrf_resp_t (*rf_channel)(uint8_t channel);

    nrf_resp_t (*rf_data_rate)(rf_data_rate_params_t data_rate);

    nrf_resp_t (*rf_power)(rf_power_params_t rf_power);

    nrf_resp_t (*send_package)(const void *tx_packet, size_t size);

    nrf_resp_t (*read_payload)(void *rx_packet, size_t size);

    nrf_resp_t (*is_packet)(uint8_t *rx_p_no, uint32_t ms_timeout);

    nrf_resp_t (*start_listen)(void);

    nrf_resp_t (*stop_listen)(void);

    nrf_resp_t (*ready_to_transmit)(void);
} nrf_client_t;

nrf_resp_t nrf_driver_create_client(nrf_client_t *client);

#endif //NRF24L01_H
