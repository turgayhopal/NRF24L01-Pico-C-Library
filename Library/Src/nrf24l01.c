//
// Created by Turgay on 17.11.2023.
//

#include <pico/binary_info/code.h>
#include "nrf24l01.h"

nrf_driver_t nrf_driver = {
        .spi_control.baudrate = 7000000,
        .spi_control.instance = spi0,
        .pin_control.sck = 2,
        .pin_control.mosi = 3,
        .pin_control.miso = 4,
        .pin_control.csn = 5,
        .pin_control.ce = 6,
        .device_mode = STANDBY_I,
        .nrf_control.rf_channel = 0x01,
        .nrf_control.payload_size = 2,
        .nrf_control.rx_tx_config = PRIM_RX_PTX,
        .nrf_control.address_width = ADDR_WIDTH_5_BYTES,
        .nrf_control.crc_length_config = CRC_ENC_1_BYTE,
        .nrf_control.crc_state = CRC_ENABLE,
        .nrf_control.lna_gain_state = LNA_GAIN_ENABLE,
        .nrf_control.mask_max_rt_interrupt_state = MASK_MAX_RT_ENABLE,
        .nrf_control.mask_tx_ds_interrupt_state = MASK_TX_DS_ENABLE,
        .nrf_control.mask_rx_dr_interrupt_state = MASK_RX_DR_DISABLE,
        .nrf_control.power_state = POWER_UP,
        .nrf_control.retransmit_count = AUTO_RT_COUNT_10,
        .nrf_control.retransmit_delay = AUTO_RT_DELAY_250,
        .nrf_control.rf_data_rate = RF_DATA_RATE_2Mbps,
        .nrf_control.rf_pll_state = RF_PLL_LOCK_DISABLE,
        .nrf_control.rf_power = RF_POWER_0_dBm,
        .nrf_control.pipe_0_state = PIPE_0_ENABLE,
        .nrf_control.pipe_1_state = PIPE_1_ENABLE,
        .nrf_control.pipe_2_state = PIPE_2_DISABLE,
        .nrf_control.pipe_3_state = PIPE_3_DISABLE,
        .nrf_control.pipe_4_state = PIPE_4_DISABLE,
        .nrf_control.pipe_5_state = PIPE_5_DISABLE,
        .nrf_control.pipe_0_ack_state = AUTO_ACK_P0_ENABLE,
        .nrf_control.pipe_1_ack_state = AUTO_ACK_P1_ENABLE,
        .nrf_control.pipe_2_ack_state = AUTO_ACK_P2_ENABLE,
        .nrf_control.pipe_3_ack_state = AUTO_ACK_P3_ENABLE,
        .nrf_control.pipe_4_ack_state = AUTO_ACK_P4_ENABLE,
        .nrf_control.pipe_5_ack_state = AUTO_ACK_P5_ENABLE,
        .pipe_0_addr.addr_0 = 0xE7,
        .pipe_0_addr.addr_1 = 0xE7,
        .pipe_0_addr.addr_2 = 0xE7,
        .pipe_0_addr.addr_3 = 0xE7,
        .pipe_0_addr.addr_4 = 0xE7,
        .pipe_1_addr.addr_0 = 0xC2,
        .pipe_1_addr.addr_1 = 0xC2,
        .pipe_1_addr.addr_2 = 0xC2,
        .pipe_1_addr.addr_3 = 0xC2,
        .pipe_1_addr.addr_4 = 0xC2,
        .pipe_2_addr.addr_0 = 0xC3,
        .pipe_3_addr.addr_0 = 0xC4,
        .pipe_4_addr.addr_0 = 0xC5,
        .pipe_5_addr.addr_0 = 0xC6,
        .tx_addr.addr_0 = 0xE7,
        .tx_addr.addr_1 = 0xE7,
        .tx_addr.addr_2 = 0xE7,
        .tx_addr.addr_3 = 0xE7,
        .tx_addr.addr_4 = 0xE7,
};

void csn_low(void) {
    gpio_put(nrf_driver.pin_control.csn, 0);
}

void csn_high(void) {
    gpio_put(nrf_driver.pin_control.csn, 1);
}

void ce_low(void) {
    gpio_put(nrf_driver.pin_control.ce, 0);
}

void ce_high(void) {
    gpio_put(nrf_driver.pin_control.ce, 1);
}

void nrf_write_register(register_map_t reg, uint8_t *buffer, size_t size) {
    csn_low();
    uint8_t reg_buff[1];
    reg_buff[0] = reg | W_REGISTER;
    spi_write_blocking(nrf_driver.spi_control.instance, reg_buff, 1);
    spi_write_blocking(nrf_driver.spi_control.instance, buffer, size);
    csn_high();
}

void nrf_read_register(register_map_t reg, uint8_t *buffer, size_t size) {
    csn_low();
    uint8_t reg_buff[1];
    reg_buff[0] = reg & R_REGISTER;
    spi_write_blocking(nrf_driver.spi_control.instance, reg_buff, 1);
    spi_read_blocking(nrf_driver.spi_control.instance, 0, buffer, size);
    csn_high();
}

void nrf_flush_tx_fifo(void) {
    csn_low();
    sleep_us(2);
    spi_write_blocking(nrf_driver.spi_control.instance, (uint8_t[]) {FLUSH_TX}, 1);
    sleep_us(2);
    csn_high();
}

void nrf_flush_rx_fifo(void) {
    csn_low();
    sleep_us(2);
    spi_write_blocking(nrf_driver.spi_control.instance, (uint8_t[]) {FLUSH_RX}, 1);
    sleep_us(2);
    csn_high();
}

nrf_resp_t nrf_write_and_validate_reg(register_map_t reg) {

    bool readable_reg_st = false;
    int write_size = 1;

    nrf_resp_t resp = NRF_CONFIGURE_OK;
    uint8_t rx_buff[5];
    uint8_t tx_buff[5];
    switch (reg) {
        case CONFIG:
            tx_buff[0] = nrf_driver.nrf_control.rx_tx_config |
                         nrf_driver.nrf_control.power_state |
                         nrf_driver.nrf_control.crc_length_config |
                         nrf_driver.nrf_control.crc_state |
                         nrf_driver.nrf_control.mask_max_rt_interrupt_state |
                         nrf_driver.nrf_control.mask_tx_ds_interrupt_state |
                         nrf_driver.nrf_control.mask_rx_dr_interrupt_state;
            break;
        case EN_AA:
            tx_buff[0] = nrf_driver.nrf_control.pipe_0_ack_state |
                         nrf_driver.nrf_control.pipe_1_ack_state |
                         nrf_driver.nrf_control.pipe_2_ack_state |
                         nrf_driver.nrf_control.pipe_3_ack_state |
                         nrf_driver.nrf_control.pipe_4_ack_state |
                         nrf_driver.nrf_control.pipe_5_ack_state;
            break;
        case EN_RX_ADDR:
            tx_buff[0] = nrf_driver.nrf_control.pipe_0_state |
                         nrf_driver.nrf_control.pipe_1_state |
                         nrf_driver.nrf_control.pipe_2_state |
                         nrf_driver.nrf_control.pipe_3_state |
                         nrf_driver.nrf_control.pipe_4_state |
                         nrf_driver.nrf_control.pipe_5_state;
            break;
        case SETUP_AW:
            tx_buff[0] = nrf_driver.nrf_control.address_width;
            break;
        case SETUP_RE_TR:
            tx_buff[0] = nrf_driver.nrf_control.retransmit_delay |
                         nrf_driver.nrf_control.retransmit_count;
            break;
        case RF_CH:
            tx_buff[0] = nrf_driver.nrf_control.rf_channel & 0x7F; // Only support 7 bit
            break;
        case RF_SETUP:
            tx_buff[0] = nrf_driver.nrf_control.lna_gain_state |
                         nrf_driver.nrf_control.rf_power |
                         nrf_driver.nrf_control.rf_data_rate |
                         nrf_driver.nrf_control.rf_pll_state;
            break;
        case RX_ADDR_P0:
            write_size = 5;
            tx_buff[0] = nrf_driver.pipe_0_addr.addr_0;
            tx_buff[1] = nrf_driver.pipe_0_addr.addr_1;
            tx_buff[2] = nrf_driver.pipe_0_addr.addr_2;
            tx_buff[3] = nrf_driver.pipe_0_addr.addr_3;
            tx_buff[4] = nrf_driver.pipe_0_addr.addr_4;
            break;
        case RX_ADDR_P1:
            write_size = 5;
            tx_buff[0] = nrf_driver.pipe_1_addr.addr_0;
            tx_buff[1] = nrf_driver.pipe_1_addr.addr_1;
            tx_buff[2] = nrf_driver.pipe_1_addr.addr_2;
            tx_buff[3] = nrf_driver.pipe_1_addr.addr_3;
            tx_buff[4] = nrf_driver.pipe_1_addr.addr_4;
            break;
        case RX_ADDR_P2:
            tx_buff[0] = nrf_driver.pipe_2_addr.addr_0;
            break;
        case RX_ADDR_P3:
            tx_buff[0] = nrf_driver.pipe_3_addr.addr_0;
            break;
        case RX_ADDR_P4:
            tx_buff[0] = nrf_driver.pipe_4_addr.addr_0;
            break;
        case RX_ADDR_P5:
            tx_buff[0] = nrf_driver.pipe_5_addr.addr_0;
            break;
        case TX_ADDR:
            write_size = 5;
            tx_buff[0] = nrf_driver.tx_addr.addr_0;
            tx_buff[1] = nrf_driver.tx_addr.addr_1;
            tx_buff[2] = nrf_driver.tx_addr.addr_2;
            tx_buff[3] = nrf_driver.tx_addr.addr_3;
            tx_buff[4] = nrf_driver.tx_addr.addr_4;
            break;
        case RX_PW_P0:
            if (nrf_driver.nrf_control.pipe_0_state == PIPE_0_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case RX_PW_P1:
            if (nrf_driver.nrf_control.pipe_1_state == PIPE_1_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case RX_PW_P2:
            if (nrf_driver.nrf_control.pipe_2_state == PIPE_2_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case RX_PW_P3:
            if (nrf_driver.nrf_control.pipe_3_state == PIPE_3_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case RX_PW_P4:
            if (nrf_driver.nrf_control.pipe_4_state == PIPE_4_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case RX_PW_P5:
            if (nrf_driver.nrf_control.pipe_5_state == PIPE_5_ENABLE) {
                if (nrf_driver.nrf_control.payload_size < 32) {
                    tx_buff[0] = nrf_driver.nrf_control.payload_size & 0x1F;
                } else {
                    tx_buff[0] = 0x20;
                }
            } else {
                tx_buff[0] = 0x00;
            }
            break;
        case STATUS:
            tx_buff[0] = STATUS_RESET;
            break;
        case FIFO_STATUS:
            // Only Readable
            readable_reg_st = true;
            break;
        case OBSERVE_TX:
            // Only Readable
            readable_reg_st = true;
            break;
        case CD:
            // Only Readable
            readable_reg_st = true;
            break;
    }

    if (readable_reg_st) {
        resp = NRF_CONFIGURE_ERR;
    } else {
        if (write_size == 1) {
#ifdef DEBUG_NRF24
            printf("Tx Buff - Addr 0x%02X : 0x%02X\r\n", reg, tx_buff[0]);
#endif
            nrf_write_register(reg, tx_buff, write_size);
            sleep_us(1);
            nrf_read_register(reg, rx_buff, 1);
#ifdef DEBUG_NRF24
            printf("Rx Buff : \t      0x%02X\r\n", rx_buff[0]);
#endif
            if (rx_buff[0] != tx_buff[0]) {
                resp = NRF_CONFIGURE_ERR;
            }
#ifdef DEBUG_NRF24
            printf("------------------------------------------------\r\n");
#endif
        } else {
#ifdef DEBUG_NRF24
            printf("Tx Buff - Addr 0x%02X : ", reg);
            for (uint8_t i = 0; i < write_size; i++) {
                printf("0x%02X ", tx_buff[i]);
            }
            printf("\r\n");
#endif
            nrf_write_register(reg, tx_buff, write_size);
            sleep_us(1);
            nrf_read_register(reg, rx_buff, write_size);
#ifdef DEBUG_NRF24
            printf("Rx Buff : \t      ");
#endif
            for (uint8_t i = 0; i < write_size; i++) {
#ifdef DEBUG_NRF24
                printf("0x%02X ", rx_buff[i]);
#endif
                if (tx_buff[i] == rx_buff[i]) {
                    resp = NRF_CONFIGURE_OK;
                } else {
                    resp = NRF_CONFIGURE_ERR;
                    break;
                }
            }
#ifdef DEBUG_NRF2
            printf("\r\n");
            printf("------------------------------------------------\r\n");
#endif
        }
    }

    return resp;
}


void nrf_write_payload(const void *tx_packet, size_t size) {
    csn_low();
    sleep_us(2);
    spi_write_blocking(nrf_driver.spi_control.instance, (uint8_t[]) {W_TX_PAYLOAD}, 1);
    spi_write_blocking(nrf_driver.spi_control.instance, (uint8_t *) tx_packet, size);
    sleep_us(2);
    csn_high();
}

nrf_resp_t nrf_read_payload(void *rx_packet, size_t size) {
    csn_low();
    sleep_us(2);
    spi_write_blocking(nrf_driver.spi_control.instance, (uint8_t[]) {R_RX_PAYLOAD}, 1);
    spi_read_blocking(nrf_driver.spi_control.instance, 0, rx_packet, size);
    sleep_us(2);
    csn_high();
    nrf_flush_rx_fifo();
    return NRF_OK;
}


nrf_resp_t nrf_io_configure(pin_control_t *pin_control, spi_control_t *spi_control) {

    // Get Pointer Address of Pin Control on
    pin_control_t *pins = &(nrf_driver.pin_control);

    // store user_pins in global nrf_driver struct
    *pins = *pin_control;

    // Get Pointer Address of Pin Control on
    spi_control_t *spi = &(nrf_driver.spi_control);

    // store user_pins in global nrf_driver struct
    *spi = *spi_control;

    // Init Output IO
    gpio_init(nrf_driver.pin_control.csn);
    gpio_init(nrf_driver.pin_control.ce);
    gpio_set_dir(nrf_driver.pin_control.csn, GPIO_OUT);
    gpio_set_dir(nrf_driver.pin_control.ce, GPIO_OUT);

    // Init Spi
    spi_init(nrf_driver.spi_control.instance, nrf_driver.spi_control.baudrate);
    gpio_set_function(nrf_driver.pin_control.sck, GPIO_FUNC_SPI);
    gpio_set_function(nrf_driver.pin_control.miso, GPIO_FUNC_SPI);
    gpio_set_function(nrf_driver.pin_control.mosi, GPIO_FUNC_SPI);

    return NRF_CONFIGURE_OK;
}

nrf_resp_t nrf_irq_clear(void) {
    return nrf_write_and_validate_reg(STATUS);
}

nrf_resp_t nrf_check_irq(nrf_irq_check_t check_irq, uint8_t *pipe_number, uint32_t timeout) {
    nrf_resp_t resp;

    uint8_t rx_buff[1];
    uint32_t start_tick = time_us_32();
    while (1) {
        if (time_us_32() - start_tick < timeout * 1000) {
            nrf_read_register(STATUS, rx_buff, 1);
            if (rx_buff[0] & check_irq) {
                if (pipe_number != NULL) {
                    *pipe_number = 0x07 & rx_buff[0];
                }
                resp = NRF_OK;
                if (check_irq != TX_FULL) {
                    nrf_irq_clear();
                }
                break;
            }
        } else {
            resp = NRF_ERR;
            break;
        }
    }
    return resp;
}

nrf_resp_t nrf_set_mode(nrf_device_mode_t mode) {

    switch (mode) {
        case STANDBY_I:
            nrf_driver.nrf_control.power_state = POWER_UP;
            nrf_driver.nrf_control.rx_tx_config = PRIM_RX_PTX;
            ce_low();
            break;
        case STANDBY_II:
            nrf_driver.nrf_control.power_state = POWER_UP;
            nrf_driver.nrf_control.rx_tx_config = PRIM_RX_PTX;
            ce_low();
            break;
        case RX_MODE:
            nrf_driver.nrf_control.power_state = POWER_UP;
            nrf_driver.nrf_control.rx_tx_config = PRIM_RX_PRX;
            sleep_us(130); // NRF24L01+ enters RX Mode after 130μS
            ce_high();
            break;
        case TX_MODE:
            nrf_driver.nrf_control.power_state = POWER_UP;
            nrf_driver.nrf_control.rx_tx_config = PRIM_RX_PTX;
            ce_low();
            break;
        case POWER_DOWN_MODE:
            nrf_driver.nrf_control.power_state = POWER_DOWN;
            nrf_driver.nrf_control.rx_tx_config = PRIM_RX_PTX;
            ce_low();
            break;
    }
    nrf_flush_tx_fifo();
    nrf_flush_rx_fifo();

    return nrf_write_and_validate_reg(CONFIG);;
}

void nrf_transmit_pulse(void) {
    ce_high();
    sleep_us(15); // High Push CE Pin > 10us
    ce_low();
    // nrf_set_mode(STANDBY_I);
}

nrf_resp_t is_packet(uint8_t *rx_p_no, uint32_t ms_timeout) {
    return nrf_check_irq(RX_DR, rx_p_no, ms_timeout);
}

nrf_resp_t nrf_send_package(const void *tx_packet, size_t size) {
    nrf_write_payload(tx_packet, size);
    nrf_transmit_pulse();
    if (nrf_check_irq(MAX_RT, NULL, 10) == NRF_OK) {
        if (nrf_check_irq(TX_FULL, NULL, 10) == NRF_OK) {
            nrf_flush_tx_fifo();
        }
    }
    return nrf_check_irq(TX_DS, NULL, 10);
}

nrf_resp_t nrf_start_listen(void) {
    nrf_irq_clear();
    return nrf_set_mode(RX_MODE);
}

nrf_resp_t nrf_stop_listen(void) {
    return nrf_set_mode(STANDBY_I);
}

nrf_resp_t nrf_set_standby_mode(void) {
    return nrf_set_mode(STANDBY_I);
}

nrf_resp_t set_auto_retransmission(re_tr_delay_params_t delay, re_tr_count_params_t count) {
    nrf_driver.nrf_control.retransmit_delay = delay;
    nrf_driver.nrf_control.retransmit_count = count;
    return nrf_write_and_validate_reg(SETUP_RE_TR);
}

nrf_resp_t set_rf_channel(uint8_t channel) {
    nrf_driver.nrf_control.rf_channel = channel & 0x7F;
    return nrf_write_and_validate_reg(RF_CH);
}

nrf_resp_t set_rf_data_rate(rf_data_rate_params_t data_rate) {
    nrf_driver.nrf_control.rf_data_rate = data_rate;
    return nrf_write_and_validate_reg(RF_SETUP);
}

nrf_resp_t set_rf_power(rf_power_params_t rf_power) {
    nrf_driver.nrf_control.rf_power = rf_power;
    return nrf_write_and_validate_reg(RF_SETUP);
}

nrf_resp_t set_payload_size(size_t size) {
    return nrf_write_and_validate_reg(SETUP_AW);
}

nrf_resp_t set_pipe_0_destination(pipe_0_addr_t *pipe_addr) {
    pipe_0_addr_t *addr = &(nrf_driver.pipe_0_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P0);
}

nrf_resp_t set_pipe_1_destination(pipe_1_addr_t *pipe_addr) {
    pipe_1_addr_t *addr = &(nrf_driver.pipe_1_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P1);
}

nrf_resp_t set_pipe_2_destination(pipe_2_addr_t *pipe_addr) {
    pipe_2_addr_t *addr = &(nrf_driver.pipe_2_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P2);
}

nrf_resp_t set_pipe_3_destination(pipe_3_addr_t *pipe_addr) {
    pipe_3_addr_t *addr = &(nrf_driver.pipe_3_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P3);
}

nrf_resp_t set_pipe_4_destination(pipe_4_addr_t *pipe_addr) {
    pipe_4_addr_t *addr = &(nrf_driver.pipe_4_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P4);
}

nrf_resp_t set_pipe_5_destination(pipe_5_addr_t *pipe_addr) {
    pipe_5_addr_t *addr = &(nrf_driver.pipe_5_addr);
    if (pipe_addr != NULL)
        *addr = *pipe_addr;
    return nrf_write_and_validate_reg(RX_ADDR_P5);
}

nrf_resp_t set_tx_destination(tx_addr_t *tx_addr) {
    tx_addr_t *addr = &(nrf_driver.tx_addr);
    if (tx_addr != NULL)
        *addr = *tx_addr;
    return nrf_write_and_validate_reg(TX_ADDR);
}

nrf_resp_t nrf_initialise(nrf_control_t *nrf_control) {

    // Get Pointer Address of Nrf Control on
    nrf_control_t *control = &(nrf_driver.nrf_control);

    // store user_pins in global nrf_driver struct
    if (nrf_control != NULL) {
        *control = *nrf_control;
    }

    nrf_resp_t resp = NRF_CONFIGURE_OK;

    ce_low();
    csn_high();
    sleep_ms(10);

    resp &= nrf_write_and_validate_reg(CONFIG);
    resp &= nrf_write_and_validate_reg(EN_AA);
    resp &= nrf_write_and_validate_reg(EN_RX_ADDR);
    resp &= nrf_write_and_validate_reg(SETUP_AW);
    resp &= nrf_write_and_validate_reg(SETUP_RE_TR);
    resp &= nrf_write_and_validate_reg(RF_CH);
    resp &= nrf_write_and_validate_reg(RF_SETUP);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P0);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P1);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P2);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P3);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P4);
    resp &= nrf_write_and_validate_reg(RX_ADDR_P5);
    resp &= nrf_write_and_validate_reg(TX_ADDR);
    resp &= nrf_write_and_validate_reg(RX_PW_P0);
    resp &= nrf_write_and_validate_reg(RX_PW_P1);
    resp &= nrf_write_and_validate_reg(RX_PW_P2);
    resp &= nrf_write_and_validate_reg(RX_PW_P3);
    resp &= nrf_write_and_validate_reg(RX_PW_P4);
    resp &= nrf_write_and_validate_reg(RX_PW_P5);

    nrf_flush_rx_fifo();
    nrf_flush_tx_fifo();

    return resp;
}

nrf_resp_t nrf_driver_create_client(nrf_client_t *client) {

    client->io_configure = nrf_io_configure;
    client->initialise = nrf_initialise;

    client->rx_0_destination = set_pipe_0_destination;
    client->rx_1_destination = set_pipe_1_destination;
    client->rx_2_destination = set_pipe_2_destination;
    client->rx_3_destination = set_pipe_3_destination;
    client->rx_4_destination = set_pipe_4_destination;
    client->rx_5_destination = set_pipe_5_destination;
    client->tx_destination = set_tx_destination;

    client->payload_size = set_payload_size;
    client->auto_retransmission = set_auto_retransmission;
    client->rf_channel = set_rf_channel;
    client->rf_data_rate = set_rf_data_rate;
    client->rf_power = set_rf_power;
    client->read_payload = nrf_read_payload;
    client->send_package = nrf_send_package;
    client->is_packet = is_packet;
    client->start_listen = nrf_start_listen;
    client->stop_listen = nrf_stop_listen;
    client->ready_to_transmit = nrf_set_standby_mode;
    return NRF_CONFIGURE_OK;
}