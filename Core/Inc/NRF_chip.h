
#ifndef INC_NRF_CHIP_H_
#define INC_NRF_CHIP_H_

#include <stdint.h>
#include  <stdio.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "NRF24_conf.h"

#define PLD_S 32 //payload size for rf

void init_nrf(void);

#endif /* INC_NRF_CHIP_H_ */
