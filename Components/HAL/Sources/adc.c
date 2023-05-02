/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
#include "adc.h"
#include "netx_drv.h"
#include "par_def.h"
#include "mpwm.h"

#define ADC_VREF                      PAR_ADC_VREF_VOLTS
#define ADC_SHUNT_RESISTANCE          PAR_SHUNT_RESISTANCE_OHM
#define ADC_GAIN_CSA                  PAR_CURRENT_AMP_GAIN
#define ADC_CURRENT_GAIN              ((1.0f)/(ADC_GAIN_CSA * ADC_SHUNT_RESISTANCE))
#define ADC_DIG_TO_VOLT               (ADC_VREF / 4096.0f)
#define ADC_DIG_TO_AMP                (float)(ADC_DIG_TO_VOLT * ADC_CURRENT_GAIN)
#define ADC_VOLTAGE_GAIN              PAR_DCBUS_VOLTAGE_GAIN
#define ADC_VOLTAGE_OFFSET            PAR_DCBUS_VOLTAGE_OFFSET_DIGITS
#define ADC_DIG_TO_DCVOLT             (float)(ADC_VOLTAGE_GAIN * ADC_DIG_TO_VOLT)
#define ADC_TRIGGER                   ((MPWM_HALF_PERIOD * 2) - 2U)

/*!
 * \details   Array contains ADC sample.
 */
uint16_t adc_results[ADC_RESULT_ARRAY_MAX_SIZE];

/*!
 * \brief     Holds measured current offset values.
 * \details   Values are in digits.
 */
static adcCurrentOffsetT s_CurrentOffset;

/*!
 * \details   Sample time is set to 0.5 microseconds.
 *            Measurements done only once and each mpwm period has to be started again.
 *            Triggers for current measurements are set to end of mpwm period.
 */
void adc_Initialize()
{
  DRV_ADC_HANDLE_T tADC0 = DRV_ADC_INITIALIZER;
  DRV_ADC_HANDLE_T tADC1 = DRV_ADC_INITIALIZER;
  DRV_ADC_HANDLE_T tADC2 = DRV_ADC_INITIALIZER;
  DRV_ADC_HANDLE_T tADC3 = DRV_ADC_INITIALIZER;
  DRV_ADC_SEQ_HANDLE_T tSeq0 = DRV_ADC_SEQ_INITIALIZER;
  DRV_ADC_SEQ_HANDLE_T tSeq1 = DRV_ADC_SEQ_INITIALIZER;
  DRV_ADC_SEQ_HANDLE_T tSeq2 = DRV_ADC_SEQ_INITIALIZER;
  DRV_ADC_SEQ_HANDLE_T tSeq3 = DRV_ADC_SEQ_INITIALIZER;

  /******************************* ISENA ************************************************/
  DRV_ADC_Driver_Init(&tADC0); /* Initialize and enable ADC driver */
  tSeq0.tConfiguration.eDeviceID = DRV_ADC_SEQ_DEVICE_ID_ADC0; /* Assign sequencer to ADC0 */
  tSeq0.tConfiguration.eOperationMode = DRV_OPERATION_MODE_DMA; /* Save result to memory*/
  tSeq0.tConfiguration.eChannelTrackingTime[1] = 0x32; /* Maximum sample time 0.5us at channel 1 */
  tSeq0.tConfiguration.eContinuousMode = DRV_ADC_SEQ_CONTINUOUS_MODE_DISABLED; /* Measure once */
  tSeq0.tConfiguration.eBaseAdr = (DRV_ADC_SEQ_BASE_ADDRESS_E) &(adc_results[ADC_ISENA_VAL]); /* Address of results */
  tSeq0.tConfiguration.eVrefVdd3 = DRV_ADC_SEQ_VREF_VDD3_ENABLED; /* USe VDD3(3.3V) as Reference Voltage */
  tSeq0.tConfiguration.tMeasurement[0].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq0.tConfiguration.tMeasurement[0].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_1; /* Select channel IN1 for ISENA #L5*/
  tSeq0.tConfiguration.tMeasurement[0].eAdrOffset = 0u;
  tSeq0.tConfiguration.tMeasurement[0].eTrigger = ADC_TRIGGER;
  DRV_ADC_Seq_Init(&tSeq0, &tADC0);

  /**************************** ISENB  **********************************************/
  DRV_ADC_Driver_Init(&tADC1); /* Initialize and enable ADC driver */
  tSeq1.tConfiguration.eDeviceID = DRV_ADC_SEQ_DEVICE_ID_ADC1; /* Assign sequencer to ADC1 */
  tSeq1.tConfiguration.eOperationMode = DRV_OPERATION_MODE_DMA; /* Save result to memory */
  tSeq1.tConfiguration.eChannelTrackingTime[1] = 0x32; /* Maximum sample time 0.5us at channel 0 */
  tSeq1.tConfiguration.eContinuousMode = DRV_ADC_SEQ_CONTINUOUS_MODE_DISABLED;
  tSeq1.tConfiguration.eBaseAdr = (DRV_ADC_SEQ_BASE_ADDRESS_E) &(adc_results[ADC_ISENB_VAL]); /* Address of results */
  tSeq1.tConfiguration.eVrefVdd3 = DRV_ADC_SEQ_VREF_VDD3_ENABLED; /* USe VDD3(3.3V) as Reference Voltage */
  tSeq1.tConfiguration.tMeasurement[0].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq1.tConfiguration.tMeasurement[0].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_1; /* Select channel IN1 for ISENB #L4*/
  tSeq1.tConfiguration.tMeasurement[0].eAdrOffset = 0u;
  tSeq1.tConfiguration.tMeasurement[0].eTrigger = ADC_TRIGGER;
  DRV_ADC_Seq_Init(&tSeq1, &tADC1);

  /***************************** AIM, VSENA, POT, NTC, VSENVM ****************************/
  DRV_ADC_Driver_Init(&tADC2); /* Initialize and enable ADC driver */
  tSeq2.tConfiguration.eDeviceID = DRV_ADC_SEQ_DEVICE_ID_ADC2; /* Assign sequencer to ADC2 */
  tSeq2.tConfiguration.eOperationMode = DRV_OPERATION_MODE_DMA; /* Save result to memory */
  tSeq2.tConfiguration.eChannelTrackingTime[1] = 0x32; /* Maximum sample time 1us at channel 1 */
  tSeq2.tConfiguration.eChannelTrackingTime[2] = 0x32; /* Maximum sample time 1us at channel 2 */
  tSeq2.tConfiguration.eChannelTrackingTime[3] = 0x32; /* Maximum sample time 1us at channel 3 */
  tSeq2.tConfiguration.eChannelTrackingTime[4] = 0x32; /* Maximum sample time 1us at channel 4 */
  tSeq2.tConfiguration.eChannelTrackingTime[6] = 0x32; /* Maximum sample time 1us at channel 6 */
  tSeq2.tConfiguration.eContinuousMode = DRV_ADC_SEQ_CONTINUOUS_MODE_DISABLED; /* Measure once */
  tSeq2.tConfiguration.eBaseAdr = (DRV_ADC_SEQ_BASE_ADDRESS_E) &(adc_results[ADC_VSENVM_VAL]); /* Address of results */
  tSeq2.tConfiguration.eVrefVdd3 = DRV_ADC_SEQ_VREF_VDD3_ENABLED; /* USe VDD3(3.3V) as Reference Voltage */
  tSeq2.tConfiguration.tMeasurement[0].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq2.tConfiguration.tMeasurement[0].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_4; /* Select channel IN4 for VSENVM #H7*/
  tSeq2.tConfiguration.tMeasurement[0].eAdrOffset = 0u;
  tSeq2.tConfiguration.tMeasurement[0].eTrigger = ADC_TRIGGER;
  tSeq2.tConfiguration.tMeasurement[1].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq2.tConfiguration.tMeasurement[1].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_2; /* Select channel IN2 for POT #H10*/
  tSeq2.tConfiguration.tMeasurement[1].eAdrOffset = 1u;
  tSeq2.tConfiguration.tMeasurement[2].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq2.tConfiguration.tMeasurement[2].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_3; /* Select channel IN3 for VSENA #K9*/
  tSeq2.tConfiguration.tMeasurement[2].eAdrOffset = 2u;
  tSeq2.tConfiguration.tMeasurement[3].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq2.tConfiguration.tMeasurement[3].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_1; /* Select channel IN6 for AIM #L7*/
  tSeq2.tConfiguration.tMeasurement[3].eAdrOffset = 3u;
  tSeq2.tConfiguration.tMeasurement[4].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq2.tConfiguration.tMeasurement[4].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_6; /* Select channel IN6 for NTC #L8*/
  tSeq2.tConfiguration.tMeasurement[4].eAdrOffset = 4u;
  DRV_ADC_Seq_Init(&tSeq2, &tADC2);

  /***************************** VSENB, VSENC, ISENC, AI   **********************************/
  DRV_ADC_Driver_Init(&tADC3); /* Initialize and enable ADC driver */
  tSeq3.tConfiguration.eDeviceID = DRV_ADC_SEQ_DEVICE_ID_ADC3; /* Assign sequencer to ADC3 */
  tSeq3.tConfiguration.eOperationMode = DRV_OPERATION_MODE_DMA; /* Save result to memory */
  tSeq3.tConfiguration.eChannelTrackingTime[1] = 0x32; /* Maximum sample time 0.5us at channel 1 */
  tSeq3.tConfiguration.eChannelTrackingTime[2] = 0x32; /* Maximum sample time 0.5us at channel 2 */
  tSeq3.tConfiguration.eChannelTrackingTime[3] = 0x32; /* Maximum sample time 0.5us at channel 3 */
  tSeq3.tConfiguration.eChannelTrackingTime[5] = 0x32; /* Maximum sample time 0.5us at channel 5 */
  tSeq3.tConfiguration.eContinuousMode = DRV_ADC_SEQ_CONTINUOUS_MODE_DISABLED; /* Measure once*/
  tSeq3.tConfiguration.eBaseAdr = (DRV_ADC_SEQ_BASE_ADDRESS_E) &(adc_results[ADC_ISENC_VAL]);/* Address of results */
  tSeq3.tConfiguration.eVrefVdd3 = DRV_ADC_SEQ_VREF_VDD3_ENABLED; /* USe VDD3(3.3V) as Reference Voltage */
  tSeq3.tConfiguration.tMeasurement[0].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq3.tConfiguration.tMeasurement[0].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_2; /* Select channel IN2 for ISENC #K7*/
  tSeq3.tConfiguration.tMeasurement[0].eAdrOffset = 0u;
  tSeq3.tConfiguration.tMeasurement[0].eTrigger = ADC_TRIGGER;
  tSeq3.tConfiguration.tMeasurement[1].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq3.tConfiguration.tMeasurement[1].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_5; /* Select channel IN5 for VSENC #H6*/
  tSeq3.tConfiguration.tMeasurement[1].eAdrOffset = 1u;
  tSeq3.tConfiguration.tMeasurement[2].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq3.tConfiguration.tMeasurement[2].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_3; /* Select channel IN3 for VSENB #K6*/
  tSeq3.tConfiguration.tMeasurement[2].eAdrOffset = 2u;
  tSeq3.tConfiguration.tMeasurement[3].eEnable = DRV_ADC_SEQ_MEAS_ENABLE; /* Enable one measurement */
  tSeq3.tConfiguration.tMeasurement[3].eInputChannel = DRV_ADC_SEQ_MEAS_INPUT_CHANNEL_1; /* Select channel IN1 for AI #L6*/
  tSeq3.tConfiguration.tMeasurement[3].eAdrOffset = 3u;
  DRV_ADC_Seq_Init(&tSeq3, &tADC3);

  DRV_ADC_Seq_Start(&tSeq0); /* Start sampling */
  DRV_ADC_Seq_Start(&tSeq1); /* Start sampling */
  DRV_ADC_Seq_Start(&tSeq2); /* Start sampling */
  DRV_ADC_Seq_Start(&tSeq3); /* Start sampling */
}

void adc_Start()
{
  madc->madc_start = 0xF;
}

void adc_WaitToFinish()
{
  while(madc_seq0->madc_seq_status_b.adc_half_clock_cycle != 0)
  {
  }
  while(madc_seq1->madc_seq_status_b.adc_half_clock_cycle != 0)
  {
  }
  while(madc_seq2->madc_seq_status_b.adc_half_clock_cycle != 0)
  {
  }
  while(madc_seq3->madc_seq_status_b.adc_half_clock_cycle != 0)
  {
  }
}

Math_Vector_UVW_T adc_GetMeasuredCurrents()
{
  Math_Vector_UVW_T l_Return;
  {
    l_Return.f_u = (float) (ADC_DIG_TO_AMP * (-s_CurrentOffset.u16_CurrentOffset_u + adc_results[ADC_ISENA_VAL]));
    l_Return.f_v = (float) (ADC_DIG_TO_AMP * (-s_CurrentOffset.u16_CurrentOffset_v + adc_results[ADC_ISENB_VAL]));
    l_Return.f_w = (float) (ADC_DIG_TO_AMP * (-s_CurrentOffset.u16_CurrentOffset_w + adc_results[ADC_ISENC_VAL]));
  }
  return l_Return;
}

adcCurrentOffsetT adc_MeasureCurrentOffset()
{
  adcCurrentOffsetT l_Result;
  l_Result.u16_CurrentOffset_u = adc_results[ADC_ISENA_VAL];
  l_Result.u16_CurrentOffset_v = adc_results[ADC_ISENB_VAL];
  l_Result.u16_CurrentOffset_w = adc_results[ADC_ISENC_VAL];
  return l_Result;
}

void adc_SetCurrentOffsets(adcCurrentOffsetT *aps_MeasuredCurrOffsets)
{
  s_CurrentOffset.u16_CurrentOffset_u = aps_MeasuredCurrOffsets->u16_CurrentOffset_u;
  s_CurrentOffset.u16_CurrentOffset_v = aps_MeasuredCurrOffsets->u16_CurrentOffset_v;
  s_CurrentOffset.u16_CurrentOffset_w = aps_MeasuredCurrOffsets->u16_CurrentOffset_w;
}

float adc_GetVSenVM()
{
  float l_Return;
  l_Return = ((float) (adc_results[ADC_VSENVM_VAL] + ADC_VOLTAGE_OFFSET) * ADC_DIG_TO_DCVOLT );
  return l_Return;
}

uint16_t adc_GetPotValue()
{
  uint16_t l_Return;
  l_Return = adc_results[ADC_POT_VAL];
  return l_Return;
}
