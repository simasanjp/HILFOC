#ifndef __FOC_APPL_DRV_H_
#define __FOC_APPL_DRV_H_

#include <stdbool.h>
#include <stdint.h>
#include "netx_drv_def.h"




DRV_STATUS_E FOC_APPL_init();
DRV_STATUS_E FOC_APPL_deInit();

uint16_t FOC_APPL_setSpeedSetpoint(uint16_t uSpeed);
uint16_t FOC_APPL_getSpeed();

DRV_STATUS_E FOC_APPL_checkGDUError();

#endif /* #ifndef __FOC_APPL_DRV_H_ */

