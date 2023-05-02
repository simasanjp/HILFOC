

#ifndef GATEDRIVER_DRV8323_HINC
#define GATEDRIVER_DRV8323_HINC

/**** includes ****************************************************************/

#include "drv8323_registers.h"


/**** type definitions and macros *********************************************/

/**
 * \file gatedriver_drv8323.h
 *
 * This component provides a method to startup or disable the driver before using it.
 */

#define GATEDRIVER_DRV8323_ERROR_REGISTER_SIZE 2U
#define GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE 5U

typedef enum {
  GATEDRIVER_DRV8323_INIT,
  GATEDRIVER_DRV8323_WRITE_CONFIGURATION_REGISTERS,
  GATEDRIVER_DRV8323_READ_CONFIGURATION_REGISTERS,
  GATEDRIVER_DRV8323_READ_STATUS_REGISTERS,
  GATEDRIVER_DRV8323_WAIT_FOR_SLEEP,
  GATEDRIVER_DRV8323_WAIT_FOR_WAKEUP,
  GATEDRIVER_DRV8323_DONE,
  GATEDRIVER_DRV8323_FAILED
} GatedriverDrv8323InternalStateT;

typedef struct GatedriverCommonErrorS {
  uint8_t u8_UnderVoltagePresent;    /**< transient under voltage on either bridge voltage or driver bootstrap voltage, not critical and transient */
  uint8_t u8_OverVoltagePresent;     /**< transient over  voltage on either bridge voltage or driver bootstrap voltage, not critical and transient */
  uint8_t u8_OverCurrentPresent;     /**< over current error, detected either via phase current measurement or vds error reporting, false positives should be removed, transient but might be critical during pre check */
  uint8_t u8_DesatPresent;           /**< driver has detected a desaturation error. */
  uint8_t u8_InternalError;          /**< all remaining errors, they are classified fatal and not transient and will cause the device to be exchanged */
  uint8_t u8_OverTemperaturePresent; /**< the driver was shutdown because of an driver internal overtemperature condition */
} GatedriverCommonErrorT;

typedef enum
{
  GATEDRIVER_OPERATION_ENABLE,
  GATEDRIVER_OPERATION_DISABLE,
  GATEDRIVER_OPERATION_GET_STATUS
} GatedriverOperationT;

typedef struct GatedriverDrv8323ErrorS {
  uint8_t                  u8_DriverIsUnreachableViaSpi;
  GatedriverCommonErrorT s_CommonErrors;
} GatedriverDrv8323ErrorT;

typedef union GateDriverDrv8323ErrorRegistersU {
  uint16_t au16_Data[GATEDRIVER_DRV8323_ERROR_REGISTER_SIZE];
  struct {
    GatedriverDrv8323_FaultStatusRegister1T s_FSR1;
    GatedriverDrv8323_FaultStatusRegister2T s_FSR2;
  }        s_Register;
} GateDriverDrv8323ErrorRegistersT;

typedef union GateDriverDrv8323ConfigurationU {
  uint16_t au16_Data[GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE];
  struct {
    GatedriverDrv8323_DriverControlRegisterT s_DCR;
    GatedriverDrv8323_GateDriveHST           s_GDHS;
    GatedriverDrv8323_GateDriveLST           s_GDLS;
    GatedriverDrv8323_OCPControlT            s_OCP;
    GatedriverDrv8323_CSAControlT            s_CSA;
  }        s_Register;
} GateDriverDrv8323ConfigurationT;

typedef struct GatedriverDrv8323StateS {
  GatedriverDrv8323InternalStateT  e_MainState;       /**< main state machine enumeration of the post check */
  GatedriverDrv8323ErrorT          s_Error;           /**< errors occurred */
  GatedriverOperationT             e_TargetOperation;
  GateDriverDrv8323ConfigurationT  s_Configuration;
  GateDriverDrv8323ErrorRegistersT s_Status;
  uint8_t                          u8_Index;
} GatedriverDrv8323StateT;

/**** variable definitions ****************************************************/

/**** variable declarations ***************************************************/

/**** functions declarations **************************************************/

/**
 * \brief gatedriver startup function
 *
 * This function is called during startup and may take some time to complete.
 */
void GatedriverDrv8323_Init(GatedriverDrv8323StateT *aps_State);

/**
 * \brief gatedriver_Drv8323 state preparation
 *
 * This function initializes the GatedriverDrv8323StateT state.
 * It must be called before calling GatedriverDrv8323_PwmPeriod.
 *
 * It is intended to be short and called during PWM-IRQ
 *
 * @param[in] aps_State gate driver state
 * @param[in] ae_Operation the driver shall be enabled or disabled or queried for it's error status
 */
void GatedriverDrv8323_Prepare(GatedriverDrv8323StateT *aps_State, GatedriverOperationT ae_Operation);

/**
 * \brief pre check worker function
 *
 * This function is intended for operation in the pwm irq. It sets adc trigger points and
 * configures the pwm. It should be called each pwm cycles when it is active. When
 * it is not active, it does not have to be called.
 *
 * @param[in,out] aps_State pre check state
 * @param[in] au1_DriverError the user indicates that the gdu reports a driver, 0 no error, 1 an error occurred
 */
void GatedriverDrv8323_PwmPeriod(GatedriverDrv8323StateT *aps_State);


/**
 * \brief report back error reason after having performed a GET_STATUS-query on the driver
 *
 * \warning this status is only valid after having queried the driver with a GATEDRIVER_OPERATION_GET_STATUS
 * operation
 *
 * When the test has failed, the error reason can be read back.
 * It cannot be assured that one of these bits is set. The test
 * can fail for other reasons like under voltage.
 *
 * @param[in] ap_State post check state
 * @param[out] ap_Errors errors recorded during the test
 */
void GatedriverDrv8323_GetErrors(const GatedriverDrv8323StateT *aps_State,
                                 GatedriverDrv8323ErrorT *aps_Errors);




#endif /* GATEDRIVER_Drv8323_HINC */
