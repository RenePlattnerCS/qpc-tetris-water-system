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
static void listen();

uint8_t tx_addr[5] = { '0', '0', '0', '0', '1' };
uint16_t data = 0;

uint8_t dataR[PLD_S];

void init_nrf(void)
{
	  csn_high();
	  ce_high();

	  //HAL_Delay(5);
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

	  nrf24_open_tx_pipe(tx_addr);
	  nrf24_open_rx_pipe(0, tx_addr);
	  ce_high();
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

static void listen()
{
	 nrf24_listen();

	  if(nrf24_data_available()){
		nrf24_receive(dataR, sizeof(dataR));
	  }
	  HAL_Delay(5);
	  data = nrf24_uint8_t_to_type(dataR, sizeof(dataR));
}
