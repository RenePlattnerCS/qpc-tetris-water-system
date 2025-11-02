/*
 * NRF_chip.c
 *
 *  Created on: Nov 2, 2025
 *      Author: NeerW
 */

#include "NRF_chip.h"
#include "main_app.h"
#include "bsp.h"

extern SPI_HandleTypeDef hspi1;
void NRF24_TestCommunication(void);
void verify_rx_mode(void);


uint8_t tx_addr[5] = { '0', '0', '0', '0', '1' };
	uint16_t data = 0;

	uint8_t dataR[PLD_S];
void init_nrf(void)
{
	uint8_t tx_addr2[5] = { '0', '0', '0', '0', '1' };

	  csn_high();
	  ce_high();

	  BSP_delayMs(5);

	  ce_low();

	  nrf24_init();
	  uint8_t reg = nrf24_r_reg(0x00,1 );


	  nrf24_auto_ack_all(auto_ack);
	  nrf24_en_ack_pld(disable);
	  nrf24_dpl(disable);

	  nrf24_set_crc(en_crc, _2byte);

	  nrf24_tx_pwr(_0dbm);
	  nrf24_data_rate(_1mbps);
	  nrf24_set_channel(76);
	  nrf24_set_addr_width(5);

	  nrf24_set_rx_dpl(0, disable);
	  nrf24_set_rx_dpl(1, disable);
	  nrf24_set_rx_dpl(2, disable);
	  nrf24_set_rx_dpl(3, disable);
	  nrf24_set_rx_dpl(4, disable);
	  nrf24_set_rx_dpl(5, disable);

	  nrf24_pipe_pld_size(0, PLD_S);

	  nrf24_auto_retr_delay(4);
	  nrf24_auto_retr_limit(10);

	  nrf24_open_tx_pipe(tx_addr2);
	  nrf24_open_rx_pipe(0, tx_addr2);
	  ce_high();
	  //nrf24_listen();
	  nrf24_clear_rx_dr();
	      nrf24_clear_tx_ds();
	      nrf24_clear_max_rt();

	      // Flush RX FIFO
	      nrf24_flush_rx();

	      // Power up
	      nrf24_pwr_up();
	      BSP_delayMs(2);

	      // Start listening (this might set MASK_RX_DR=1)
	      nrf24_listen();
	      // Enable RX_DR interrupt
	          uint8_t config_val = 0x0F;  // RX mode, powered up, interrupts enabled
	          nrf24_w_reg(0x00, &config_val, 1);
}

uint8_t check_data(void)
{
	uint8_t reg_dt = nrf24_r_reg(FIFO_STATUS, 1);
	if(!(reg_dt & (1 << RX_EMPTY))){
		return 1;
	}
	return 0;

}
void receive(uint8_t *data, uint8_t size)
{

	uint8_t cmd = R_RX_PAYLOAD;

	csn_low();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, spi_w_timeout);
	HAL_SPI_Receive(&hspi1, data, size, spi_r_timeout);
	csn_high();

	nrf24_clear_rx_dr();

}

void listen(void)
{
	 nrf24_listen();

	  if(nrf24_data_available()){
		nrf24_receive(dataR, sizeof(dataR));
	  }
	  //HAL_Delay(5);
	  BSP_delayMs(5);
	  data = nrf24_uint8_t_to_type(dataR, sizeof(dataR));
	  printf("wait\n");
}






void NRF24_TestCommunication(void)
{
    uint8_t reg_addr = 0x00; // CONFIG register
    uint8_t cmd = 0x00 | 0x00; // R_REGISTER + reg_addr
    uint8_t reg_val = 0;

    csn_low(); // Select NRF
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, &reg_val, 1, 100);
    csn_high(); // Deselect NRF

    printf("NRF CONFIG register: 0x%02X\n", reg_val);

    // Optional: read STATUS register too
    cmd = 0x07; // STATUS register address
    csn_low();
    uint8_t rreg_cmd = 0x00 | cmd;
    HAL_SPI_Transmit(&hspi1, &rreg_cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, &reg_val, 1, 100);
    csn_high();

    printf("NRF STATUS register: 0x%02X\n", reg_val);
}



void verify_rx_mode(void)
{
    uint8_t config = nrf24_r_reg(0x00, 1);
    uint8_t status = nrf24_r_status();
    uint8_t en_rxaddr = nrf24_r_reg(0x02, 1);
    uint8_t rx_pw_p0 = nrf24_r_reg(0x11, 1);

    // CONFIG should be 0x0F (RX mode, powered up, interrupts enabled)
    if (config != 0x0F) {
        while(1);  // Wrong config
    }

    // EN_RXADDR should have bit 0 set (pipe 0 enabled)
    if (!(en_rxaddr & 0x01)) {
        while(1);  // Pipe 0 not enabled
    }

    // RX_PW_P0 should be PLD_S (payload size for pipe 0)
    if (rx_pw_p0 != PLD_S) {
        while(1);  // Wrong payload size
    }

    // CE pin should be HIGH (listening)
    uint8_t ce_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    if (ce_state != GPIO_PIN_SET) {
        while(1);  // CE not high
    }

    // All checks passed
}
