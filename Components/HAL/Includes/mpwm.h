/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file mpwm.h
 *
 * \brief  MPWM abstraction.
 */
#ifndef __MPWM_H_
#define __MPWM_H_
#include "par_def.h"
#include "netx_drv.h"

/*!
 * \brief     MPWM Period in ticks.
 */
#define MPWM_HALF_PERIOD         (mpwm_app->mpwm_cnt_max_s_b.val)             /* Half Period Ticks : From shadow register */
/*!
 * \brief     Event counter top register value.
 */
#define EVENT_COUNTER_TOP   (1U)      /* MPWM interrupt will occur every EVENT_COUNTER_TOP + 1 period */

typedef uint32_t MPWM_SPI_FAULT_E;

enum MPWM_SPI_FAULT_STATUS_Etag
{
  MPWM_SPI_STATUS_NO_FAULT       = 0,
  //Fault Register 1
  MPWM_SPI_FAULT_STATUS_FAULT    = 0x1u<<10,                           //Logic OR of FAULT status registers. Mirrors nFAULT pin.
  MPWM_SPI_FAULT_STATUS_VDS_OCP  = 0x1u<<9,                            //Indicates VDS monitor overcurrent fault condition
  MPWM_SPI_FAULT_STATUS_GDF      = 0x1u<<8,                            //Indicates gate drive fault condition
  MPWM_SPI_FAULT_STATUS_UVLO     = 0x1u<<7,                            //Indicates undervoltage lockout fault condition
  MPWM_SPI_FAULT_STATUS_OTSD     = 0x1u<<6,                            //Indicates overtemperature shutdown
  MPWM_SPI_FAULT_STATUS_VDS_HA   = 0x1u<<5,                            //Indicates VDS overcurrent fault on the A high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VDS_LA   = 0x1u<<4,                            //Indicates VDS overcurrent fault on the A low-side MOSFET
  MPWM_SPI_FAULT_STATUS_VDS_HB   = 0x1u<<3,                            //Indicates VDS overcurrent fault on the B high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VDS_LB   = 0x1u<<2,                            //Indicates VDS overcurrent fault on the B low-side MOSFET
  MPWM_SPI_FAULT_STATUS_VDS_HC   = 0x1u<<1,                            //Indicates VDS overcurrent fault on the C high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VDS_LC   = 0x1u,                               //Indicates VDS overcurrent fault on the C low-side MOSFET
  //Fault Register 2
  MPWM_SPI_FAULT_STATUS_SA_OC    = 0x1u<<21,                           //Indicates overcurrent on phase A sense amplifier (DRV8323xS)
  MPWM_SPI_FAULT_STATUS_SB_OC    = 0x1u<<20,                           //Indicates overcurrent on phase B sense amplifier (DRV8323xS)
  MPWM_SPI_FAULT_STATUS_SC_OC    = 0x1u<<19,                           //Indicates overcurrent on phase C sense amplifier (DRV8323xS)
  MPWM_SPI_FAULT_STATUS_OTW      = 0x1u<<18,                           //Indicates overtemperature warning
  MPWM_SPI_FAULT_STATUS_CPUV     = 0x1u<<17,                           //Indicates charge pump undervoltage fault condition
  MPWM_SPI_FAULT_STATUS_VGS_HA   = 0x1u<<16,                           //Indicates gate drive fault on the A high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VGS_LA   = 0x1u<<15,                           //Indicates gate drive fault on the A low-side MOSFET
  MPWM_SPI_FAULT_STATUS_VGS_HB   = 0x1u<<14,                           //Indicates gate drive fault on the B high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VGS_LB   = 0x1u<<13,                           //Indicates gate drive fault on the B low-side MOSFET
  MPWM_SPI_FAULT_STATUS_VGS_HC   = 0x1u<<12,                           //Indicates gate drive fault on the C high-side MOSFET
  MPWM_SPI_FAULT_STATUS_VGS_LC   = 0x1u<<11,                           //Indicates gate drive fault on the C low-side MOSFET
};

/*!
 * \brief     Initialize mpwm with triangle count mode.
 */
void mpwm_Initialize();

/*!
 * \brief     Start mpwm.
 */
void mpwm_Start();

/*!
 * \brief     Retrieve the fault registers from the GDU via SPI
 *            first 11 bits - Fault Register 1. Next 16 bits - Fault Register 2
 */
void mpwm_GetFaultErrorRegisters(uint32_t * mpwm_us_FaultRegs);

/*!
 * \brief     Stop mpwm.
 */
void mpwm_Stop();

/*!
 * \brief     Set duty Cycle for mpwm_ch0 and mpwm_ch1 pair.
 */
void mpwm_SetDutyCycleA(uint16_t au16_OnTicks);

/*!
 * \brief     Set duty Cycle for mpwm_ch2 and mpwm_ch3 pair.
 */
void mpwm_SetDutyCycleB(uint16_t au16_OnTicks);

/*!
 * \brief     Set duty Cycle for mpwm_ch4 and mpwm_ch5 pair.
 */
void mpwm_SetDutyCycleC(uint16_t au16_OnTicks);

/*!
 * \brief     Enable all low side.
 */
void mpwm_EnableLowSide();

/*!
 * \brief     Enable Gate Driver Unit.
 */
void mpwm_EnableGDU();

/*!
 * \brief     Disable Gate Driver Unit.
 */
void mpwm_DisableGDU();


/*!
 * \brief     Disable PWM output.
 */
void mpwm_DisablePWM();

/*!
 * \brief     Enable PWM output.
 */
void mpwm_EnablePWM();
/*!
 * \brief     Acknowledge mpwm interrupt.
 */
void mpwm_AcknowledgeIRQ();

/*!
 *  \brief      Synch mpwm period
 */
void mpwm_Synch();

 void mpwm_SS_Set();

 void mpwm_SS_Clear();
#endif /* #ifndef __MPWM_H_ */
