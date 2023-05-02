/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file foc.h
 *
 * \brief  Field-Oriented-Control implementation.
 */
#ifndef __FOC_H_
#define __FOC_H_
#include "mathematics.h"
#include "netx_drv.h"


/**
 * \brief FOC state machine states
 */
typedef enum FocStates
{
    FOC_IDLE,                 /**< idle mode, pwm switched off */
    FOC_CURRENT_CONTROL,      /**< field oriented current control loop */
    FOC_CURRENT_OFFSET,       /**< current offset calibration */
    FOC_PWM_PATTERN,          /**< direct control of the PWM hw module */
    FOC_VOLTAGE_CONTROL,      /**< field oriented voltage control, measurement like current control, but no control loop for id, iq */
    FOC_FAILED = -1,          /**< failed mode, this is no request mode */
} FocStateMachine_MainStateT;

/**
 * \brief Foc current offset state machine states
 */
typedef enum CurrentOffsetStates
{
    CURRENT_OFFSET_INIT,      /**< init current offset measurement */
    CURRENT_OFFSET_MEASURE,   /**< measure current offset */
    CURRENT_OFFSET_CALC,      /**< calculate current offset by averaging*/
    CURRENT_OFFSET_DONE,      /**< current offset measurement done */
    CURRENT_OFFSET_FAILED,    /**< current offset measurement failed */
} FocCurrentOffset_StateT;

/**
 * \brief Foc current offset parameters.
 */
typedef struct CurrentOffsetParameterS
{
  uint8_t   u8_NumberOfMeasurements;  /**< \brief number of pwm ticks ( PwmN ) to enable the lowside */
  uint32_t  u32_SumCurrOffset_u;      /**< \brief Sum of current offset measurements on phase u */
  uint32_t  u32_SumCurrOffset_v;      /**< \brief Sum of current offset measurements on phase v */
  uint32_t  u32_SumCurrOffset_w;      /**< \brief Sum of current offset measurements on phase w */
  uint16_t  u16_CurrOffsetCounter;    /**< \brief Wait to reinit after failed state. */
} Foc_CurrOffsetParameterT;

/**
 * \brief Duty-vector in UVW domain
 */
typedef struct DutyTicks
{
    uint16_t u16_u; /**< phase U ticks */
    uint16_t u16_v; /**< phase V ticks */
    uint16_t u16_w; /**< phase W ticks */
} Foc_Duty_UVW_T;

/**
 * \brief DC-bus ripple elimination parameters.
 */
typedef struct DCBusRippleParameters
{
    float     f_DCBusVoltage;                  /**< Measured DCBus Volatege */
    float     f_Sum;                           /**< Sum to use in moving average filter */
    float     f_DiscardVoltage;                /**< DCBus measurement to be discarded used in moving average filter  */
    uint8_t   u8_Index;                        /**< Index of moving average filter */
    float     f_DCBusArray[4];                /**< Holds last captured DCBus voltage values */
} Foc_DCBusRipple_ParameterT;





/**
 * \brief Initialize FOC.
 */
void Foc_Init();
/*!
 * \brief    Main FOC state machine.
 */
void Foc_StateMachine();
/*!
 * \brief    FOC state machine transition function.
 */
void Foc_TransitState(FocStateMachine_MainStateT as_State);
/**
 * \brief     Get current offset state.
 * @return    Current offset state.
 */
FocCurrentOffset_StateT Foc_GetCurrentOffsetState();
/*!
 * \brief   Set requested current vector in dq frame.
 */
void Foc_SetRequestCurrent(Math_Vector_DQ_T as_ReqCurrent);
#if (PAR_USE_OPENLOOP == 1)
/*!
 * \brief   Set requested voltage vector in dq frame.
 */
void Foc_SetRequestVoltage(Math_Vector_DQ_T as_ReqVoltage);
#endif

/**
 * \brief     Get requested voltage vector in dq frame.
 * @return    Requested voltage vector in DQ frame.
 */
Math_Vector_DQ_T Foc_GetRequestVoltage();


#endif /* #ifndef __FOC_H_ */
