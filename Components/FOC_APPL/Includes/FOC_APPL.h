#ifndef __FOC_APPL_DRV_H_
#define __FOC_APPL_DRV_H_

#include <stdbool.h>
#include <stdint.h>
#include "netx_drv_def.h"

typedef struct FOC_APPL_DATA_Ttag * FOC_APPL_HANDLE_T;

DRV_STATUS_E FOC_APPL_init(FOC_APPL_HANDLE_T * ptHnd);
DRV_STATUS_E FOC_APPL_deInit(FOC_APPL_HANDLE_T tHnd);

uint16_t FOC_APPL_setSpeedSetpoint(uint16_t uSpeed, FOC_APPL_HANDLE_T tHnd);
uint16_t FOC_APPL_getSpeed(FOC_APPL_HANDLE_T tHnd);

DRV_STATUS_E FOC_APPL_checkGDUError();

#endif /* #ifndef __FOC_APPL_DRV_H_ */

