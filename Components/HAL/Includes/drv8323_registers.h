/*
 * drv8323_registers.h
 *
 *  Created on: Nov 3, 2021
 *      Author: Admin
 */

#ifndef COMPONENTS_HAL_INCLUDES_DRV8323_REGISTERS_H_
#define COMPONENTS_HAL_INCLUDES_DRV8323_REGISTERS_H_
/**** type definitions and macros *********************************************/
#include "netx_drv.h"
/**
 * \file gatedriver_drv8323_registers.h
 *
 * This file contains the register definitions of the drv8323
 */

typedef enum
{
  DRV8323_REG_FSR_1 = 0x0,
  DRV8323_REG_FSR_2 = 0x1,
  DRV8323_REG_DCR   = 0x2,
  DRV8323_REG_GDHS  = 0x3,
  DRV8323_REG_GDLS  = 0x4,
  DRV8323_REG_OCP   = 0x5,
  DRV8323_REG_CSA   = 0x6
} GatedriverDrv8323AddrT;

typedef struct {
  uint16_t u16_VDS_LC:1;  /* Indicates VDS overcurrent fault on the C low-side MOSFET */
  uint16_t u16_VDS_HC:1;  /* Indicates VDS overcurrent fault on the C high-side MOSFET */
  uint16_t u16_VDS_LB:1;  /* Indicates VDS overcurrent fault on the B low-side MOSFET */
  uint16_t u16_VDS_HB:1;  /* Indicates VDS overcurrent fault on the B high-side MOSFET */
  uint16_t u16_VDS_LA:1;  /* Indicates VDS overcurrent fault on the A low-side MOSFET */
  uint16_t u16_VDS_HA:1;  /* Indicates VDS overcurrent fault on the A high-side MOSFET */
  uint16_t u16_OTSD:1;    /* Indicates overtemperature shutdown */
  uint16_t u16_UVLO:1;    /* Indicates undervoltage lockout fault condition */
  uint16_t u16_GDF:1;     /* Indicates gate drive fault condition */
  uint16_t u16_VDS_OCP:1; /* Indicates VDS monitor overcurrent fault condition */
  uint16_t u16_FAULT:1; /* Logic OR of FAULT status registers. Mirrors nFAULT pin.*/
  uint16_t u16_Unused:4;  /* unused */
} GatedriverDrv8323_FaultStatusRegister1T;

typedef struct {
  uint16_t u16_VGS_LC:1; /* Indicates gate drive fault on the C low-side MOSFET */
  uint16_t u16_VGS_HC:1; /* Indicates gate drive fault on the C high-side MOSFET */
  uint16_t u16_VGS_LB:1; /* Indicates gate drive fault on the B low-side MOSFET */
  uint16_t u16_VGS_HB:1; /* Indicates gate drive fault on the B high-side MOSFET */
  uint16_t u16_VGS_LA:1; /* Indicates gate drive fault on the A low-side MOSFET */
  uint16_t u16_VGS_HA:1; /* Indicates gate drive fault on the A high-side MOSFET */
  uint16_t u16_CPUV:1;   /* Indicates charge pump undervoltage fault condition */
  uint16_t u16_OTW:1;    /* Indicates over temperature warning */
  uint16_t u16_SC_OC:1;  /* Indicates over current on phase C sense amplifier (DRV8323xS) */
  uint16_t u16_SB_OC:1;  /* Indicates over current on phase B sense amplifier (DRV8323xS) */
  uint16_t u16_SA_OC:1;  /* Indicates over current on phase A sense amplifier (DRV8323xS) */
  uint16_t u16_Unused:5; /* unused */
} GatedriverDrv8323_FaultStatusRegister2T;

typedef struct {
  uint16_t u16_CLR_FLT:1; /* Write a 1 to this bit to clear latched fault bits. This bit automatically resets after being writen. */
  uint16_t u16_BRAKE:1;   /* Write a 1 to this bit to turn on all three low-side MOSFETs. This bit is ORed with the INLC (BRAKE) input in 1x PWM mode. */
  uint16_t u16_COAST:1;   /* Write a 1 to this bit to put all MOSFETs in the Hi-Z state */
  uint16_t u16_1PWM_DIR:1; /* In 1x PWM mode this bit is ORed with the INHC (DIR) input */
  uint16_t u16_1PWM_COM:1; /* 0b = 1x PWM mode uses synchronous rectification, 1b = 1x PWM mode uses asynchronous rectification */
  uint16_t u16_PWM_MODE:2; /* 00b = 6x PWM Mode, 01b = 3x PWM mode, 10b = 1x PWM mode, 11b = Independent PWM mode */
  uint16_t u16_OTW_REP:1;  /* 0b = OTW is not reported on nFAULT or the FAULT bit, 1b = OTW is reported on nFAULT and the FAULT bit */
  uint16_t u16_DIS_GDF:1;  /* 0b = Gate drive fault is enabled, 1b = Gate drive fault is disabled */
  uint16_t u16_DIS_CPUV:1; /* 0b =Charge pump undervoltage lockout fault is enabled, 1b = Charge pump undervoltage lockout fault is disabled */
  uint16_t u16_Unused:6;   /* unused */
} GatedriverDrv8323_DriverControlRegisterT;

typedef struct {
  uint16_t u16_IDRIVEN_HS:4; /* drive current negative starting at 20mA */
  uint16_t u16_IDRIVEP_HS:4; /* drive current positive starting at 10mA */
  uint16_t u16_LOCK:3; /* Write 110b to lock the settings by ignoring further register writes except to these bits and address 0x02h bits 0-2. Writing any sequence other than 110b has no effect when unlocked. Write 011b to this register to unlock all registers. Writing any sequence other than 011b has no effect when locked. */
  uint16_t u16_Unused:5; /* unused */
} GatedriverDrv8323_GateDriveHST;

typedef struct {
  uint16_t u16_IDRIVEN_LS:4; /* drive current negative starting at 20mA */
  uint16_t u16_IDRIVEP_LS:4; /* drive current positive starting at 10mA */
  uint16_t u16_TDRIVE:2;     /* 00b = 500-ns peak gate-current drive time 01b = 1000-ns peak gate-current drive time 10b = 2000-ns peak gate-current drive time 11b = 4000-ns peak gate-current drive time */
  uint16_t u16_CBC:1;        /* Active only when OCP_MODE = 01b 0b = For VDS_OCP and SEN_OCP, the fault is cleared after tRETRY 1b = For VDS_OCP and SEN_OCP, the fault is cleared when a new PWM input is given or after tRETRY */
  uint16_t u16_Unused:5;     /* unused */
} GatedriverDrv8323_GateDriveLST;

typedef struct {
  uint16_t u16_VDS_LVL:4;
  uint16_t u16_OCP_DEG:2;   /* 00b = Overcurrent deglitch of 1 us 01b = Overcurrent deglitch of 2 us 10b = Overcurrent deglitch of 4 us 11b = Overcurrent deglitch of 8 us */
  uint16_t u16_OCP_MODE:2;  /* 00b = Overcurrent causes a latched fault, 01b = Overcurrent causes an automatic retrying fault, 10b = Overcurrent is report only but no action is taken 11b = Overcurrent is not reported and no action is taken */
  uint16_t u16_DEAD_TIME:2; /* 00b = 50-ns dead time 01b = 100-ns dead time 10b = 200-ns dead time 11b = 400-ns dead time */
  uint16_t u16_TRETRY:1;    /* 0b = VDS_OCP and SEN_OCP retry time is 4 ms, 1b = VDS_OCP and SEN_OCP retry time is 50 us */
  uint16_t u16_Unused:5;    /* unused */
} GatedriverDrv8323_OCPControlT;

typedef struct {
  uint16_t u16_SEN_LVL:2;   /* 00b = Sense OCP 0.25 V 01b = Sense OCP 0.5 V 10b = Sense OCP 0.75 V 11b = Sense OCP 1 V */
  uint16_t u16_CSA_CAL_C:1; /* 0b = Normal sense amplifier C operation 1b = Short inputs to sense amplifier C for offset calibration */
  uint16_t u16_CSA_CAL_B:1; /* 0b = Normal sense amplifier B operation 1b = Short inputs to sense amplifier C for offset calibration */
  uint16_t u16_CSA_CAL_A:1; /* 0b = Normal sense amplifier A operation 1b = Short inputs to sense amplifier C for offset calibration */
  uint16_t u16_DIS_SEN:1;   /* 0b = Sense overcurrent fault is enabled, 1b = Sense overcurrent fault is disabled */
  uint16_t u16_CSA_GAIN:2;  /* 00b = 5-V/V shunt amplifier gain 01b = 10-V/V shunt amplifier gain 10b = 20-V/V shunt amplifier gain 11b = 40-V/V shunt amplifier gain */
  uint16_t u16_LS_REF:1;    /* 0b = VDS_OCP for the low-side MOSFET is measured across SHx to SPx, 1b = VDS_OCP for the low-side MOSFET is measured across SHx to SNx */
  uint16_t u16_VREF_DIV:1;  /* 0b = Sense amplifier reference voltage is VREF (unidirectional mode), 1b = Sense amplifier reference voltage is VREF divided by 2 */
  uint16_t u16_CSA_FET:1;   /* 1b = Sense amplifier positive input is SHx (also automatically sets the LS_REF bit to 1), 0b = Sense amplifier positive input is SPx */
  uint16_t u16_Unused:5;    /* unused */
} GatedriverDrv8323_CSAControlT;

#endif /* COMPONENTS_HAL_INCLUDES_DRV8323_REGISTERS_H_ */
