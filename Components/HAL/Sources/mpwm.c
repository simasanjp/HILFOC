/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */

#include "mpwm.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#ifdef HILSCHER_CODE
#define MPWM_SPI_DATA_BITS        0x7FF        //0000011111111111
#define MPWM_SPI_ADDRESS_BITS     0x7800       //0111100000000000
#define MPWM_SPI_READ_BIT         0x8000       //1000000000000000

#define MPWM_SPI_ADDRESS_BITS_POS   11
#define MPWM_SPI_READ_BIT_POS       1

#define MPWM_SPI_READ  1
#define MPWM_SPI_WRITE 0

#define MPWM_SPI_FAULT_STATUS_REGISTER1_ADDRESS 0x0
#define MPWM_SPI_FAULT_STATUS_REGISTER2_ADDRESS 0x1

#define MPWM_SPI_MAX_TRY_SESSION  5

typedef uint16_t MPWM_SPI_PACKET_T;

static DRV_SPI_HANDLE_T tSPIHnd;

#endif

/*!
 * \brief     Previously captured synch in signal.
 */
static int16_t mpwm_s16_prevSynch;
/*!
 * \brief     Currently captured synch in signal.
 */
static int16_t mpwm_s16_currSynch;
/*!
 * \brief     Change in synch in signal.
 */
static int16_t mpwm_s16_changeSynch;
/*!
 * \details   Checks if the Gate Driver is Active(if PIO is high)
 */
bool mpwm_IsGDUEnabled();

/*!
 * \details   HS and LS are inverted by muxin settings.
 * \image   html "mpwm configuration.svg"
 */

void mpwm_SS_Set()
{

  pio_app->pio_out_b.val |= DRV_DIO_MSK_PIO_12; /* PIO12 set high*/
}

void mpwm_SS_Clear()
{
  //pio_app->pio_oe_b.val &= ~(DRV_DIO_MSK_PIO_12 ); /* PIO12 output disable*/
  pio_app->pio_out_b.val &= ~(DRV_DIO_MSK_PIO_12 ); /* PIO12 set LOW*/
}
#ifdef HILSCHER_CODE
void Create_SPI_Packet(MPWM_SPI_PACKET_T* ptSPIPacket, uint8_t ubReadBit, uint8_t ubAddress, uint16_t usData)
{
  memset(ptSPIPacket, 0, sizeof(MPWM_SPI_PACKET_T));

  if (ubReadBit)
  {
    *ptSPIPacket +=  MPWM_SPI_READ_BIT & (ubReadBit<<MPWM_SPI_READ_BIT_POS);
  }

  if (ubAddress)
  {
    *ptSPIPacket +=  MPWM_SPI_ADDRESS_BITS & (ubAddress<<MPWM_SPI_ADDRESS_BITS_POS);
  }

  if(usData)
  {
    *ptSPIPacket +=  MPWM_SPI_DATA_BITS & usData;
  }
}

void mpwm_GetFaultErrorRegisters(uint32_t * mpwm_us_FaultRegs)
{
  uint16_t usRxDataFault1 = 0;
  uint16_t usRxDataFault2 = 0;
  MPWM_SPI_PACKET_T tPacket;
  bool GDUEnabled = false;
  DRV_STATUS_E      eRet;           /** driver status declaration */
  uint8_t bCounter = 0;

  if (!mpwm_us_FaultRegs)
  {
    return;
  }

  //Read Fault Register1
  Create_SPI_Packet(&tPacket, MPWM_SPI_READ, MPWM_SPI_FAULT_STATUS_REGISTER1_ADDRESS, 0);
  GDUEnabled = mpwm_IsGDUEnabled();

  if (!GDUEnabled)
  {
    mpwm_EnableGDU();
    volatile int i;
    for (i = 0; i < 11000; i++) {} /* wait 10ms after enabling per the spec */
  }

  //Clear the Chip Select
  mpwm_SS_Clear();
  do
  {
    eRet = DRV_SPI_TransmitReceive(&tSPIHnd, (uint8_t*) &tPacket, (uint8_t*) &usRxDataFault1, sizeof(tPacket));
  }
  while((eRet != DRV_OK) | ((bCounter++) > MPWM_SPI_MAX_TRY_SESSION));
  if(eRet != DRV_OK)
  {
    /** transmit error */
    //Error_Handler();
    return;
  }

  /** wait for the end of the transfer */
  do
  {
    eRet = DRV_SPI_GetState(&tSPIHnd, 0);
  }
  while(eRet == DRV_BUSY);
  mpwm_SS_Set();

  //Read Fault Register2
  Create_SPI_Packet(&tPacket, MPWM_SPI_READ, MPWM_SPI_FAULT_STATUS_REGISTER2_ADDRESS, 0);

  mpwm_SS_Clear();
  bCounter = 0;
  do
  {
    eRet = DRV_SPI_TransmitReceive(&tSPIHnd, (uint8_t*)&tPacket, (uint8_t*)&usRxDataFault2, sizeof(tPacket));
  }
  while((eRet != DRV_OK) | ((bCounter++) > MPWM_SPI_MAX_TRY_SESSION));
  if(eRet != DRV_OK)
  {
    /** transmit error */
    //Error_Handler();
    return;
  }

  /** wait for the end of the transfer */
  do
  {
    eRet = DRV_SPI_GetState(&tSPIHnd, 0);
  }
  while(eRet == DRV_BUSY);
  mpwm_SS_Set();
  (void) DRV_IRQ_Enable();

  //disable the GDU if its initial state was disabled
  if (!GDUEnabled)
  {
    mpwm_DisableGDU();
  }

  //store both registers in a 32-bit variable
  *(uint16_t*)mpwm_us_FaultRegs = usRxDataFault1 & MPWM_SPI_DATA_BITS;
  (*mpwm_us_FaultRegs) = (*mpwm_us_FaultRegs)<<5;
  *(((uint16_t*)mpwm_us_FaultRegs) + 1) = usRxDataFault2 & MPWM_SPI_DATA_BITS;
  (*mpwm_us_FaultRegs) = (*mpwm_us_FaultRegs)>>5;
}

static void mwpm_SPI_Init()
{
  mpwm_SS_Set();

  DRV_STATUS_E eRet; /** driver status declaration */

  memset(&tSPIHnd, 0, sizeof(DRV_SPI_HANDLE_T));
  DRV_LOCK(&tSPIHnd);
  tSPIHnd.tConfiguration.eSPIDeviceID = DRV_SPI_DEVICE_ID_SPI1;
  tSPIHnd.tConfiguration.eOperationMode = DRV_OPERATION_MODE_POLL;
  tSPIHnd.tConfiguration.eFSS = DRV_SPI_FSS_NONE; // chip select
  tSPIHnd.tConfiguration.eFSSStatic = DRV_SPI_FSS_STATIC_CALLER;
  tSPIHnd.tConfiguration.uMode.value = DRV_SPI_MODE_1;
  tSPIHnd.tConfiguration.eFILTER = DRV_SPI_FILTER_INACTIVE;
  tSPIHnd.tConfiguration.eDataSize = DRV_SPI_DATA_SIZE_SELECT_16b;
  tSPIHnd.tConfiguration.eParallelism = DRV_SPI_PARALLELISM_1BIT;
  tSPIHnd.tConfiguration.eDuplex = DRV_SPI_DUPLEX_FULL;
  tSPIHnd.tConfiguration.eFrequency = DRV_SPI_FREQUENCY_6_25MHz;  //< IRQ mode in debug build
  tSPIHnd.tConfiguration.eBehaviour = DRV_SPI_BEHAVIOUR_MASTER;

  if(DRV_OK != (eRet = DRV_SPI_Init(&tSPIHnd)))
  {

    (void) DRV_SPI_DeInit(&tSPIHnd);
  }
  DRV_UNLOCK(&tSPIHnd);
  mpwm_SS_Set();
}
#endif
void mpwm_Initialize()
{
  mpwm_app->mpwm_cmd = mpwm_app_mpwm_cmd_stop_Msk; /*Before setting up mpwm make sure unit is stopped. */

  /********************mpwm_cfg************************************************/
  mpwm_app->mpwm_cfg_b.evt_cnt_top = EVENT_COUNTER_TOP; /* Used to specify the maximum value of EVT_CNT.
   * When EVT_CNT is zero and a begin of period )BOP) event occurs, an event counter zero (ECZ) event is emitted and
   * EVT_CNT is reset to evt_cnt_top. If an begin of period event occurs and EVT_CNT is not zero, EVT_CNT is decremented.
   * This mechanism can be used as an event prescaler to reduce the number of period interrupts from the MPWM module.
   * Examples: If evt_cnt_top is zero, ECZ events and BOP events will coincide. If evt_cnt_top is one, ECZ events will appear every other period
   * (half the frequency of BOP events). If evt_cnt_top is two, ECZ events will appear every third period (one third the frequency of BOP events).*/
  mpwm_app->mpwm_cfg_b.sce_src_ecz = 1u; /* 1=emit shadow copy event when event counter reaches zero*/
  mpwm_app->mpwm_cfg_b.cnt_mode = 1u; /* 00: sawtooth, 01: triangle, 10:inv sawtooth, 11: inv triangle */
  mpwm_app->mpwm_cfg_b.cnt_en_rs = 1U; /* Copy ectn on synch */
  /********************Configure for Triangle Count************************************************/
  /* mpwm_ch[0..5]_muxin_s : Channel [0...5] multiplexer input shadow register:
   * Shadow register for channel [0...5] multiplexer input register.
   */
  if(mpwm_app->mpwm_cfg_b.cnt_mode == 1u)
  {
    mpwm_app->mpwm_ch0_muxin_s_b.ab = 1u; /* above */
    mpwm_app->mpwm_ch0_muxin_s_b.bt = 0u; /* between */
    mpwm_app->mpwm_ch0_muxin_s_b.bl = 0u; /* below */
    mpwm_app->mpwm_ch1_muxin_s_b.ab = 0u;
    mpwm_app->mpwm_ch1_muxin_s_b.bt = 1u;
    mpwm_app->mpwm_ch1_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch2_muxin_s_b.ab = 1u;
    mpwm_app->mpwm_ch2_muxin_s_b.bt = 0u;
    mpwm_app->mpwm_ch2_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch3_muxin_s_b.ab = 0u;
    mpwm_app->mpwm_ch3_muxin_s_b.bt = 1u;
    mpwm_app->mpwm_ch3_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch4_muxin_s_b.ab = 1u;
    mpwm_app->mpwm_ch4_muxin_s_b.bt = 0u;
    mpwm_app->mpwm_ch4_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch5_muxin_s_b.ab = 0u;
    mpwm_app->mpwm_ch5_muxin_s_b.bt = 1u;
    mpwm_app->mpwm_ch5_muxin_s_b.bl = 0u;

    /* mpwm_ch[0..5]_muxin : Channel [0...5] multiplexer input register:
     * This register is central in determining the direct PWM output value of channel x. When (CNT  < CHX_CMP0) and
     * (CNT  < CHX_CMP1), then the direct PWM output signal is equal to CHX_MUXIN.bl (counter value is BeLow both compare values).
     * When (CNT >= CHX_CMP0) and (CNT >= CHX_CMP1), then the direct PWM output signal is equal to CHX_MUXIN.ab (counter value is
     * Above both compare values). If neither is the case, CNT is BeTween CHX_CMP0 and CHX_CMP1 and the direct PWM output signal is equal to
     * CHX_MUXIN.bt.
     */
    mpwm_app->mpwm_ch0_muxin_b.ab = 1u; /* above */
    mpwm_app->mpwm_ch0_muxin_b.bt = 0u; /* between */
    mpwm_app->mpwm_ch0_muxin_b.bl = 0u; /* below */
    mpwm_app->mpwm_ch1_muxin_b.ab = 0u;
    mpwm_app->mpwm_ch1_muxin_b.bt = 1u;
    mpwm_app->mpwm_ch1_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch2_muxin_b.ab = 1u;
    mpwm_app->mpwm_ch2_muxin_b.bt = 0u;
    mpwm_app->mpwm_ch2_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch3_muxin_b.ab = 0u;
    mpwm_app->mpwm_ch3_muxin_b.bt = 1u;
    mpwm_app->mpwm_ch3_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch4_muxin_b.ab = 1u;
    mpwm_app->mpwm_ch4_muxin_b.bt = 0u;
    mpwm_app->mpwm_ch4_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch5_muxin_b.ab = 0u;
    mpwm_app->mpwm_ch5_muxin_b.bt = 1u;
    mpwm_app->mpwm_ch5_muxin_b.bl = 0u;
  }

  if(mpwm_app->mpwm_cfg_b.cnt_mode == 0u)
  {
    mpwm_app->mpwm_ch0_muxin_s_b.ab = 0u; /* above */
    mpwm_app->mpwm_ch0_muxin_s_b.bt = 1u; /* between */
    mpwm_app->mpwm_ch0_muxin_s_b.bl = 0u; /* below */
    mpwm_app->mpwm_ch1_muxin_s_b.ab = 1u;
    mpwm_app->mpwm_ch1_muxin_s_b.bt = 0u;
    mpwm_app->mpwm_ch1_muxin_s_b.bl = 1u;
    mpwm_app->mpwm_ch2_muxin_s_b.ab = 0u;
    mpwm_app->mpwm_ch2_muxin_s_b.bt = 1u;
    mpwm_app->mpwm_ch2_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch3_muxin_s_b.ab = 1u;
    mpwm_app->mpwm_ch3_muxin_s_b.bt = 0u;
    mpwm_app->mpwm_ch3_muxin_s_b.bl = 1u;
    mpwm_app->mpwm_ch4_muxin_s_b.ab = 0u;
    mpwm_app->mpwm_ch4_muxin_s_b.bt = 1u;
    mpwm_app->mpwm_ch4_muxin_s_b.bl = 0u;
    mpwm_app->mpwm_ch5_muxin_s_b.ab = 1u;
    mpwm_app->mpwm_ch5_muxin_s_b.bt = 0u;
    mpwm_app->mpwm_ch5_muxin_s_b.bl = 1u;

    /* mpwm_ch[0..5]_muxin : Channel [0...5] multiplexer input register:
     * This register is central in determining the direct PWM output value of channel x. When (CNT  < CHX_CMP0) and (CNT  < CHX_CMP1),
     * then the direct PWM output signal is equal to CHX_MUXIN.bl (counter value is BeLow both compare values). When (CNT >= CHX_CMP0) and
     * (CNT >= CHX_CMP1), then the direct PWM output signal is equal to CHX_MUXIN.ab (counter value is ABove both compare values).
     * If neither is the case, CNT is BeTween CHX_CMP0 and CHX_CMP1 and the direct PWM output signal is equal to CHX_MUXIN.bt.
     */
    mpwm_app->mpwm_ch0_muxin_b.ab = 0u; /* above */
    mpwm_app->mpwm_ch0_muxin_b.bt = 1u; /* between */
    mpwm_app->mpwm_ch0_muxin_b.bl = 0u; /* below */
    mpwm_app->mpwm_ch1_muxin_b.ab = 1u;
    mpwm_app->mpwm_ch1_muxin_b.bt = 0u;
    mpwm_app->mpwm_ch1_muxin_b.bl = 1u;
    mpwm_app->mpwm_ch2_muxin_b.ab = 0u;
    mpwm_app->mpwm_ch2_muxin_b.bt = 1u;
    mpwm_app->mpwm_ch2_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch3_muxin_b.ab = 1u;
    mpwm_app->mpwm_ch3_muxin_b.bt = 0u;
    mpwm_app->mpwm_ch3_muxin_b.bl = 1u;
    mpwm_app->mpwm_ch4_muxin_b.ab = 0u;
    mpwm_app->mpwm_ch4_muxin_b.bt = 1u;
    mpwm_app->mpwm_ch4_muxin_b.bl = 0u;
    mpwm_app->mpwm_ch5_muxin_b.ab = 1u;
    mpwm_app->mpwm_ch5_muxin_b.bt = 0u;
    mpwm_app->mpwm_ch5_muxin_b.bl = 1u;

  }

  /*******************mpwm Period Config***************************************/
  if(mpwm_app->mpwm_cfg_b.cnt_mode == 1u)
  {
    /* mpwm_cnt_max */
    mpwm_app->mpwm_cnt_max_b.val = PAR_PWM_PERIOD / 2u; /* Triangle mode, half full period */
    mpwm_app->mpwm_cnt_max_s_b.val = PAR_PWM_PERIOD / 2u; /* Shadow register */
  }
  if(mpwm_app->mpwm_cfg_b.cnt_mode == 0u)
  {
    /* mpwm_cnt_max */
    mpwm_app->mpwm_cnt_max_b.val = PAR_PWM_PERIOD; /* Saw tooth mode, full period */
    mpwm_app->mpwm_cnt_max_s_b.val = PAR_PWM_PERIOD; /* Shadow register */
  }

  /* mpwm_cnt_ps_max */
  mpwm_app->mpwm_cnt_ps_max_b.val = 0u; /* Counter prescaler max value register 0: systimer*/
  /* mpwm_cnt_max_s */
  mpwm_app->mpwm_cnt_max_s_b.val = mpwm_app->mpwm_cnt_max_b.val; /* Counter top shadow register for next shadow copy event is the same value */

// TODO: Dead time insertion is not made. GDU unit inserts 200ns dead time.
  /* mpwm_dt : Dead time register:
   * This is the number of dead (LS and HS off) cycles (prescaled by MPWM_CNT_PS) that the dead time generator inserts when the direct
   * PWM signal changes from zero to one (rise_val) and from one to zero (fall_val). Set to zero for no dead cycles.*/
//mpwm_app->mpwm_dt_b.fall_val = 5U;
//mpwm_app->mpwm_dt_b.rise_val = 5U;    /* In steps of 100ns */
  /*******************mpwm Output Config***************************************/
  /* mpwm_ocfg */
//  mpwm_app->mpwm_ocfg_b.oe5 = 1u; /* Channel 5 output enable*/
//  mpwm_app->mpwm_ocfg_b.oe4 = 1u; /* Channel 4 output enable*/
//  mpwm_app->mpwm_ocfg_b.oe3 = 1u; /* Channel 3 output enable*/
//  mpwm_app->mpwm_ocfg_b.oe2 = 1u; /* Channel 2 output enable*/
//  mpwm_app->mpwm_ocfg_b.oe1 = 1u; /* Channel 1 output enable*/
//  mpwm_app->mpwm_ocfg_b.oe0 = 1u; /* Channel 0 output enable*/

//mpwm_app->mpwm_ocfg_b.osel5 = 1U; /* Channel 5 output selector 0: compare channel 5 direct PWM output 1: dead time generator channel 2 LS*/
//mpwm_app->mpwm_ocfg_b.osel4 = 1U; /* 1: dead time generator channel 2 HS */
//mpwm_app->mpwm_ocfg_b.osel3 = 1U; /* 1: dead time generator channel 1 LS */
//mpwm_app->mpwm_ocfg_b.osel2 = 1U; /* 1: dead time generator channel 1 HS */
//mpwm_app->mpwm_ocfg_b.osel1 = 1U; /* 1: dead time generator channel 0 LS */
//mpwm_app->mpwm_ocfg_b.osel0 = 1U; /* 1: dead time generator channel 0 HS */

  /* mpwm_dt_s: Dead time shadow register:
   Shadow register for MPWM_DT register.
   */
//mpwm_app->mpwm_dt_s_b.fall_val = mpwm_app->mpwm_dt_b.fall_val;
//mpwm_app->mpwm_dt_s_b.rise_val = mpwm_app->mpwm_dt_b.rise_val; /* No change during run time*/
  /*********************Reset all compare values**********************************/

  /* mpwm_ch[0..5]_cmp[0,1]_s : Channel [0...5] compare value [0,1] shadow register:*/

  mpwm_app->mpwm_ch0_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch0_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch1_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch1_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch2_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch2_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch3_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch3_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch4_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch4_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch5_cmp0_s_b.val = 0u;
  mpwm_app->mpwm_ch5_cmp1_s_b.val = 0u;

  /* mpwm_ch[0..5]_cmp[0,1] : Channel [0...5] compare value [0,1] register:*/

  mpwm_app->mpwm_ch0_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch0_cmp1_b.val = 0u;
  mpwm_app->mpwm_ch1_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch1_cmp1_b.val = 0u;
  mpwm_app->mpwm_ch2_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch2_cmp1_b.val = 0u;
  mpwm_app->mpwm_ch3_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch3_cmp1_b.val = 0u;
  mpwm_app->mpwm_ch4_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch4_cmp1_b.val = 0u;
  mpwm_app->mpwm_ch5_cmp0_b.val = 0u;
  mpwm_app->mpwm_ch5_cmp1_b.val = 0u;

  mpwm_app->mpwm_cmd = mpwm_app_mpwm_cmd_restart_Msk & mpwm_app_mpwm_cmd_stop_Msk; /* Writing to both will reset mpwm and it will not start*/
#ifdef HILSCHER_CODE
  mwpm_SPI_Init();
#endif

  pio_app->pio_oe_b.val |= DRV_DIO_MSK_PIO_8; /* PIO8 output enable*/
  pio_app->pio_oe_b.val |= DRV_DIO_MSK_PIO_12; /* PIO12 output enable*/
}

void mpwm_Start()
{
//TODO: implement some kind of check before starting the counter
  mpwm_app->mpwm_cmd = mpwm_app_mpwm_cmd_start_Msk; /* Start mpwm_cnt */

  /************************* IRQ enable mask:*************************************/
  mpwm_app->mpwm_irq_msk_set = mpwm_app_mpwm_irq_masked_ecz_Msk;/* Event counter reaches zero event */
//mpwm_app->mpwm_irq_msk_reset = mpwm_app_mpwm_irq_masked_bop_Msk;

  DRV_NVIC_EnableIRQ(mpwm_IRQn); /* Enable mpwm interrupt*/
  DRV_NVIC_SetPendingIRQ(mpwm_IRQn); /* Set interrupt pending */
}

void mpwm_Stop()
{
  mpwm_app->mpwm_cmd = mpwm_app_mpwm_cmd_stop_Msk; /* Stop mpwm_cnt */
//TODO: Stop IRQ???
}

/*!
 * \details   Triangle counting mode with complementary HS/LS has to be configured.
 */
void mpwm_SetDutyCycleA(uint16_t au16_OnTicks)
{
  uint16_t comp0;

  comp0 = (uint16_t) (MPWM_HALF_PERIOD - au16_OnTicks / 2u);

  /* Set using shadow registers */
  mpwm_app->mpwm_ch0_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch0_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch1_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch1_cmp1_s_b.val = 0u;

}

/*!
 * \details   Triangle counting mode with complementary HS/LS has to be configured.
 */
void mpwm_SetDutyCycleB(uint16_t au16_OnTicks)
{
  uint16_t comp0;

  comp0 = (uint16_t) (MPWM_HALF_PERIOD - au16_OnTicks / 2u);

  /* Set using shadow registers */
  mpwm_app->mpwm_ch2_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch2_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch3_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch3_cmp1_s_b.val = 0u;

}

/*!
 * \details   Triangle counting mode with complementary HS/LS has to be configured.
 */
void mpwm_SetDutyCycleC(uint16_t au16_OnTicks)
{
  uint16_t comp0;

  comp0 = (uint16_t) (MPWM_HALF_PERIOD - au16_OnTicks / 2u);

  /* Set using shadow registers */
  mpwm_app->mpwm_ch4_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch4_cmp1_s_b.val = 0u;
  mpwm_app->mpwm_ch5_cmp0_s_b.val = comp0;
  mpwm_app->mpwm_ch5_cmp1_s_b.val = 0u;

}

/*!
 * \details   For Current offset measurement.
 */
void mpwm_EnableLowSide()
{
  mpwm_SetDutyCycleA(0u);
  mpwm_SetDutyCycleB(0u);
  mpwm_SetDutyCycleC(0u);

}

/*!
 * \details   HW controlled, set PIO8 high to enable.
 */
void mpwm_EnableGDU()
{

  pio_app->pio_out_b.val |= DRV_DIO_MSK_PIO_8; /* PIO8 set high*/
  mpwm_app->mpwm_cfg_b.eci_inv = 1u;
}

/*!
 * \details   HW controlled, set PIO8 low to disable.
 */
void mpwm_DisableGDU()
{

  //pio_app->pio_oe_b.val &= ~(DRV_DIO_MSK_PIO_8); /* PIO8 output disable*/
  pio_app->pio_out_b.val &= ~(DRV_DIO_MSK_PIO_8); /* PIO8 set low*/
}
/*!
 * \brief     Disable PWM output.
 */
void mpwm_DisablePWM()
{
  mpwm_app->mpwm_ocfg_b.oe5 = 0u; /* Channel 5 output disable*/
  mpwm_app->mpwm_ocfg_b.oe4 = 0u; /* Channel 4 output disable*/
  mpwm_app->mpwm_ocfg_b.oe3 = 0u; /* Channel 3 output disable*/
  mpwm_app->mpwm_ocfg_b.oe2 = 0u; /* Channel 2 output disable*/
  mpwm_app->mpwm_ocfg_b.oe1 = 0u; /* Channel 1 output disable*/
  mpwm_app->mpwm_ocfg_b.oe0 = 0u; /* Channel 0 output disable*/
}

/*!
 * \brief     Enable PWM output.
 */
void mpwm_EnablePWM()
{
  mpwm_app->mpwm_ocfg_b.oe5 = 1u; /* Channel 5 output enable*/
  mpwm_app->mpwm_ocfg_b.oe4 = 1u; /* Channel 4 output enable*/
  mpwm_app->mpwm_ocfg_b.oe3 = 1u; /* Channel 3 output enable*/
  mpwm_app->mpwm_ocfg_b.oe2 = 1u; /* Channel 2 output enable*/
  mpwm_app->mpwm_ocfg_b.oe1 = 1u; /* Channel 1 output enable*/
  mpwm_app->mpwm_ocfg_b.oe0 = 1u; /* Channel 0 output enable*/
}
/*!
 * \details   Checks if the Gate Driver is Active(if PIO is high)
 */
bool mpwm_IsGDUEnabled()
{
  if((pio_app->pio_oe_b.val & DRV_DIO_MSK_PIO_8) && (pio_app->pio_out_b.val & DRV_DIO_MSK_PIO_8))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void mpwm_AcknowledgeIRQ()
{
  mpwm_app->mpwm_irq_raw = mpwm_app->mpwm_irq_raw;
}

void mpwm_Synch()
{
  uint16_t lu16_newPeriod;
  if(mpwm_s16_currSynch != mpwm_app->mpwm_cnt_rs_b.cnt_val / 2u)
  {
    mpwm_s16_prevSynch = mpwm_s16_currSynch;
    mpwm_s16_currSynch = mpwm_app->mpwm_cnt_rs_b.cnt_val / 2u;
    mpwm_s16_changeSynch = mpwm_s16_currSynch - mpwm_s16_prevSynch;
  }

  if((mpwm_s16_changeSynch < 250) && (mpwm_s16_changeSynch > -250)) /* 10% change is allowed */
  {
    lu16_newPeriod = mpwm_app->mpwm_cnt_max_s_b.val + mpwm_s16_changeSynch / 8U;

    if(lu16_newPeriod > (PAR_PWM_PERIOD + PAR_PWM_PERIOD / 10) / 2)
    { /* Limit period on +10%*/
      lu16_newPeriod = (PAR_PWM_PERIOD + PAR_PWM_PERIOD / 10) / 2;
    }
    if(lu16_newPeriod < (PAR_PWM_PERIOD - PAR_PWM_PERIOD / 10) / 2)
    { /* Limit period on -10% */

      lu16_newPeriod = (PAR_PWM_PERIOD - PAR_PWM_PERIOD / 10) / 2;
    }

    mpwm_app->mpwm_cnt_max_s_b.val = lu16_newPeriod;
  }

}
