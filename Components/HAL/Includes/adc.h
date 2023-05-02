/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file adc.h
 *
 * \brief  ADC abstraction.
 */
#ifndef __ADC_H_
#define __ADC_H_

#include "netx_drv_adc.h"
#include "mathematics.h"

/*!
 * \brief   Struct for current offset values.
 */
typedef struct adcCurrentOffsetS
{
  uint16_t u16_CurrentOffset_u; /**< Phase U  */
  uint16_t u16_CurrentOffset_v; /**< Phase V  */
  uint16_t u16_CurrentOffset_w; /**< Phase W  */
} adcCurrentOffsetT;

/*!
 * \brief    ADC results enum.
 */
enum ADC_RESULTS_Etag
{
  //ADC0
  ADC_ISENA_VAL = 0,
  //ADC1
  ADC_ISENB_VAL,
  //ADC2
  ADC_VSENVM_VAL,
  ADC_POT_VAL,
  ADC_VSENA_VAL,
  ADC_AIM_VAL,
  ADC_NTC_VAL,
  //ADC3
  ADC_ISENC_VAL,
  ADC_VSENC_VAL,
  ADC_VSENB_VAL,
  ADC_AI_VAL,

  ADC_RESULT_ARRAY_MAX_SIZE
};

/*!
 * \brief     Global array hold measured adc values.
 */
extern uint16_t adc_results[ADC_RESULT_ARRAY_MAX_SIZE];

/*!
 * \brief     Initialize adc and sequencers.
 */
void adc_Initialize();
/*!
 * \brief     Start adc units.
 */
void adc_Start();

/*!
 * \brief    Function to wait until adc units are done. (Not used!)
 */
void adc_WaitToFinish();

/*!
 * \brief     Measure current offset values.
 * @return    Measured current offset values in digits.
 */
adcCurrentOffsetT adc_MeasureCurrentOffset();

/*!
 * \brief     Set measured current offset values.
 */
void adc_SetCurrentOffsets(adcCurrentOffsetT *aps_MeasuredCurrOffsets);
/*!
 * \brief     Return last measured phase current values.
 * @return    Measured current offset values in UVW frame.
 */
Math_Vector_UVW_T adc_GetMeasuredCurrents();

/*!
 * \brief     Return last measured POT value.
 * @return    Measured POT value in digits.
 */
uint16_t adc_GetPotValue();

/*!
 * \brief     Return last measured DC bus voltage.
 * @return    Measured DC voltage in Volts.
 */
float adc_GetVSenVM();

#endif /* #ifndef __ADC_H_ */
