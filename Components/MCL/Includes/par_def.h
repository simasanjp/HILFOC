/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file par_def.h
 *
 * \brief  Paramater definitions.
 */
#ifndef FIRMWARE_NETX90_PAR_DEF_H
#define FIRMWARE_NETX90_PAR_DEF_H
#define DEMO_USE_POTENTIOMETER
/*!
 * \brief   Slope of Pot reading vs Speed request
 */
#define APP_POT_SPEED_SLOPE                 ((float)(PAR_MAX_SPEED_RPM - PAR_MIN_SPEED_RPM) / (float)(PAR_ADC_MAX_DIG - PAR_POT_MIN_SPEED_REQ_DIG))

/*!
 * \brief  Maximum ADC digits. For 12-bit : 4095
 */
#define PAR_ADC_MAX_DIG                     (4095U)
/*!
 * \brief Minimum Pot value for speed request.
 */
#define PAR_POT_MIN_SPEED_REQ_DIG           (200U)
/*!
 * \brief Speed request hysteresis value for Pot.
 */
#define PAR_POT_HYSTERESIS_DIG              (100U)
/*!
 * \brief  Minimum speed request.
 */
#define PAR_MIN_SPEED_RPM                   (50U)
/*!
 * \brief  Motor stopped.
 */
#define PAR_STOPPED_SPEED_RPM               (0U)
/*!
 * \brief  Motor working speed
 */
#define PAR_NOMINAL_SPEED_RPM               (1000U)

/*!
 * \brief Maximum speed request.
 */
#define PAR_MAX_SPEED_RPM                   (4000U)
/*!
 * \brief Number of measurements to calculate average current offset.
 */
#define PAR_NUMBER_OF_CURRENT_OFFSET_MEAS   (8U)
/*!
 * \brief Number of digits allowed for current offset.
 */
#define PAR_CURRENT_OFFSET_THRESHOLD_DIG    (88U)
/*!
 * \brief Maximum allowed duty cycle percentage.
 */
#define PAR_MAX_DUTY_CYCLE_PERCENT          (96.0f)
/*!
 * \brief Reference voltage for ADC.
 */
#define PAR_ADC_VREF_VOLTS                  (3.3f)
/*!
 * \brief  Shunt resistor value.
 */
#define PAR_SHUNT_RESISTANCE_OHM            (0.007f)
/*!
 * \brief Gain value of op-amp.
 */
#define PAR_CURRENT_AMP_GAIN                (20.0f)
/*!
 * \brief DC-bus Voltage gain value, calculated from voltage divider.
 */
#define PAR_DCBUS_VOLTAGE_GAIN              ((4.99f + 82.0f) / 4.99f)
/*!
 * \brief DC-bus Voltage offset value, experimentally determined.
 */
#define PAR_DCBUS_VOLTAGE_OFFSET_DIGITS     (10U)
/*!
 * \brief Phase Voltage gain value, calculated from voltage divider.
 */
#define PAR_PHASE_VOLTAGE_GAIN              ((4.99f + 82.0f) / 4.99f)
/*!
 * \brief Phase Voltage offset value.
 */
#define PAR_PHASE_VOLTAGE_OFFSET_DIGITS     (0U)
/*!
 * \brief CPU frequency in Hz.
 */
#define PAR_CPU_FREQ_HZ                     (100000000U)
/*!
 * \brief PWM frequency in Hz.
 */
#define PAR_PWM_PERIOD                     (5000U) /**< Initial pwm period */
/*!
 * \brief Application (Speed Controller) frequency.
 */
#define PAR_APP_FREQ_HZ                     (100U)
/*!
 * \brief Number of quadrature encoder slots.
 */
#define PAR_QEI_NUMBER_OF_SLOTS             (256U)
/*!
 * \brief   Use moving average filter in speed estimation.
 * \details To enable set value to 1U. Captured position  \n
 *          is filtered with a MAF of size 4.
 */
#define PAR_USE_MAF_CAPTURE                 (0U)
/*!
 * \brief Motor pole pair number.
 */
#define PAR_MOTOR_POLE_PAIRS                (3U)
/*!
 * \brief Maximum allowed phase current.
 */
#define PAR_MAX_PHASE_CURRENT_AMP           (2.5f)
/*!
 * \brief Proportional gain of speed controller.
 */
#define PAR_SPEED_CONTROLLER_KP             (0.0002f)
/*!
 * \brief Integral gain of speed controller.
 */
#define PAR_SPEED_CONTROLLER_KI             (0.000002f)
/*!
 * \brief  Proportional gain of field-weakening controller.
 */
#define PAR_FIELDWEAK_CONTROLLER_KP         (0.0f)
/*!
 * \brief Integral gain of field-weakening controller.
 */
#define PAR_FIELDWEAK_CONTROLLER_KI         (0.2f)
/*!
 * \brief   Threshold voltage vector magnitude to start field-weakening.
 * \details Value is less than maximum voltage vector magnitude \n
 *          to allow a transition region.
 */
#define PAR_FIELDWEAK_START_VOLTAGE_RATIO   (1.1f) //5320
/*!
 * \brief Proportional gain of current controller.
 */
#define PAR_CURRENT_CONTROLLER_KP           (0.01f)
/*!
 * \brief Integral gain of current controller.
 */
#define PAR_CURRENT_CONTROLLER_KI           (0.0005f)
/*!
 * \brief   To calculate speed with hall sensor.
 * \details Set to 0 to estimate speed with QEI.
 *          Set to 1 to estimate speed with sampling hall states.
 */
#define PAR_USE_HALL_SPEED                  (0)

/*!
 * \brief   Speed difference threshold to detect stall.
 */
#define STALL_DETECTION_SPEED_DIFFERENCE    (300)

/*!
 * \brief   Time it takes to set stall error.
 * \details Value is in 10ms increments.
 */
#define STALL_DETECTION_TIME_10MS           (200)

/*!
 * \brief   Rotor speed at calibration stage.
 * \details Slow speeds are generally more accurate.
 */
#define CALIBRATION_SAMPLING_RPM            (12)

/*!
 * \brief   Deviation  angle of hall sensor position when turning CW and CCW.
 * \details Angle is in digits (2^16 is 360 degrees).
 */
#define CALIBRATION_MAX_DEVIATION_ANGLE     (4000)


#endif //FIRMWARE_NETX90_PAR_DEF_H
