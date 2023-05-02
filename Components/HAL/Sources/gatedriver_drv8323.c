#include "netx_drv.h"
#include "gatedriver_drv8323.h"
#include "mpwm.h"

/**** type definitions and macros *********************************************/
#define WAIT_FOR_SLEEP_PWMPERIODS          (250u) /* 10ms */
#define WAIT_FOR_WAKEUP_PWMPERIODS         (250u) /* 10ms */
/**
 * \file gatedriver_drv8323.h
 *
 * This component provides a method to startup or disable the driver before using it.
 */
/* @formatter:off */

static GateDriverDrv8323ConfigurationT Configuration = {
  .s_Register = {
      .s_DCR = {
          .u16_DIS_CPUV = 1u,
          .u16_DIS_GDF  = 0U,
          .u16_OTW_REP  = 0U,
          .u16_PWM_MODE = 0U,
          .u16_1PWM_COM = 0U,
          .u16_1PWM_DIR = 0U,
          .u16_COAST    = 0U,
          .u16_BRAKE    = 0U,
          .u16_CLR_FLT  = 0U
      },
      .s_GDHS = {
        .u16_IDRIVEN_HS = 2U, /* 520mA*/
        .u16_IDRIVEP_HS = 2U, /* 260mA*/
        .u16_LOCK = 3U
      },
      .s_GDLS = {
        .u16_CBC = 1U,
        .u16_TDRIVE = 2U,
        .u16_IDRIVEN_LS = 2U, /* 520mA*/
        .u16_IDRIVEP_LS = 2U /* 260mA*/
      },
      .s_OCP = {
        .u16_TRETRY = 0U,
        .u16_DEAD_TIME = 3U, /* 400ns */
        .u16_OCP_MODE = 0U,
        .u16_OCP_DEG = 2U,
        .u16_VDS_LVL = 4U /* Default: 0.75V */
      },
      .s_CSA = {
        .u16_CSA_FET = 0U,
        .u16_VREF_DIV = 1U,
        .u16_LS_REF = 0U,
        .u16_CSA_GAIN = 2U,
        .u16_DIS_SEN = 0U,
        .u16_CSA_CAL_A = 0U,
        .u16_CSA_CAL_B = 0U,
        .u16_CSA_CAL_C = 0U,
        .u16_SEN_LVL = 3U /* Default: Sense OCP 1V */
      }
  }
};
/* @formatter:on */

static GatedriverDrv8323AddrT ConfigurationRegisters[GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE] = {
    DRV8323_REG_DCR, DRV8323_REG_GDHS, DRV8323_REG_GDLS, DRV8323_REG_OCP, DRV8323_REG_CSA
};

static GatedriverDrv8323AddrT StatusRegisters[GATEDRIVER_DRV8323_ERROR_REGISTER_SIZE] = {
    DRV8323_REG_FSR_1, DRV8323_REG_FSR_2
};

static DRV_SPI_HANDLE_T tSPIHnd;

void Hal_Spi_Open() {

  DRV_STATUS_E eRet; /** driver status declaration */

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
  tSPIHnd.tConfiguration.eFrequency = DRV_SPI_FREQUENCY_1_56MHz;  //< IRQ mode in debug build: 1.56MHz
  tSPIHnd.tConfiguration.eBehaviour = DRV_SPI_BEHAVIOUR_MASTER;

  if(DRV_OK != (eRet = DRV_SPI_Init(&tSPIHnd)))
  {

    (void) DRV_SPI_DeInit(&tSPIHnd);
  }
  DRV_UNLOCK(&tSPIHnd);
  mpwm_SS_Clear();
}

void Hal_Spi_Close(){
  (void) DRV_SPI_DeInit(&tSPIHnd);
  mpwm_SS_Set(); /* SPI mode on when SS set */
}


static uint16_t ReadAddress(GatedriverDrv8323AddrT ae_Addr) {
  uint16_t lu16_Value;
  uint16_t lu16_Transmit;
  DRV_STATUS_E      eRet;           /** driver status declaration */
  uint8_t lu8_Counter;

  lu16_Transmit = (((uint16_t) ae_Addr) << 11U) | 0x8000U; /* B15 : 1 to read */
  mpwm_SS_Clear();
  do
  {
    eRet = DRV_SPI_TransmitReceive(&tSPIHnd, (uint8_t*)&lu16_Transmit, (uint8_t*)&lu16_Value, sizeof(lu16_Transmit));
  }
  while((eRet != DRV_OK) | ((lu8_Counter++) > 10));
  if(eRet != DRV_OK)
  {
    /** transmit error */
    //Error_Handler();
    return 0;
  }
  /** wait for the end of the transfer */
  do
  {
    eRet = DRV_SPI_GetState(&tSPIHnd, 0);
  }
  while(eRet == DRV_BUSY);
  mpwm_SS_Set();

  return (lu16_Value & 0x07FFU);/* Don't care B15 to B11*/
}

static uint16_t WriteAddress(GatedriverDrv8323AddrT ae_Addr,
                             uint16_t au16_Value) {
  uint16_t lu16_Value;
  uint16_t lu16_Transmit;
  DRV_STATUS_E      eRet;           /** driver status declaration */
  uint8_t lu8_Counter;

  lu16_Transmit = (((uint16_t) ae_Addr) << 11U) | au16_Value;
  mpwm_SS_Clear();
  do
  {
    eRet = DRV_SPI_TransmitReceive(&tSPIHnd, (uint8_t*)&lu16_Transmit, (uint8_t*)&lu16_Value, sizeof(lu16_Transmit));

  }
  while((eRet != DRV_OK) | ((lu8_Counter++) > 10));
  if(eRet != DRV_OK)
  {
    /** transmit error */
    //Error_Handler();
    return 0;
  }
  /** wait for the end of the transfer */
  do
  {
    eRet = DRV_SPI_GetState(&tSPIHnd, 0);
  }
  while(eRet == DRV_BUSY);
  mpwm_SS_Set();
  return (lu16_Value & 0x07FFU); /* Don't care B15 to B11*/
}
void GatedriverDrv8323_Init(GatedriverDrv8323StateT *aps_State) {
  /* no operation */



}

void GatedriverDrv8323_Prepare(GatedriverDrv8323StateT *aps_State, GatedriverOperationT ae_Operation) {
  aps_State->e_TargetOperation = ae_Operation;
  aps_State->e_MainState       = GATEDRIVER_DRV8323_INIT;
}

void GatedriverDrv8323_PwmPeriod(GatedriverDrv8323StateT *aps_State)
{
  /* This function will be called every PWM period */
  uint16_t lu16_Index;

  switch (aps_State->e_MainState) {
    case GATEDRIVER_DRV8323_INIT:
      switch (aps_State->e_TargetOperation) {
        case GATEDRIVER_OPERATION_ENABLE:
          Hal_Spi_Open();
          mpwm_EnableGDU();
          aps_State->e_MainState = GATEDRIVER_DRV8323_WAIT_FOR_WAKEUP;
          aps_State->u8_Index    = 0U;
          break;
        case GATEDRIVER_OPERATION_DISABLE:
          mpwm_DisableGDU();
          aps_State->e_MainState = GATEDRIVER_DRV8323_WAIT_FOR_SLEEP;
          aps_State->u8_Index    = 0U;
          break;
        case GATEDRIVER_OPERATION_GET_STATUS:
          Hal_Spi_Open();
          aps_State->e_MainState = GATEDRIVER_DRV8323_READ_STATUS_REGISTERS;
          aps_State->u8_Index    = 0U;
          break;
      }
      break;
    case GATEDRIVER_DRV8323_WAIT_FOR_SLEEP:
      if (aps_State->u8_Index < WAIT_FOR_SLEEP_PWMPERIODS) {
        aps_State->u8_Index++;
      } else {
        aps_State->e_MainState = GATEDRIVER_DRV8323_DONE;
        aps_State->u8_Index    = 0U;
      }
      break;
    case GATEDRIVER_DRV8323_WAIT_FOR_WAKEUP:
      if (aps_State->u8_Index < WAIT_FOR_WAKEUP_PWMPERIODS) {
        aps_State->u8_Index++;
      } else {
        aps_State->e_MainState = GATEDRIVER_DRV8323_WRITE_CONFIGURATION_REGISTERS;
        aps_State->u8_Index    = 0U;
      }
      break;
    case GATEDRIVER_DRV8323_WRITE_CONFIGURATION_REGISTERS:
      if (aps_State->u8_Index < GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE) {
        /* write using spi */
        (void) WriteAddress(ConfigurationRegisters[aps_State->u8_Index],
                            Configuration.au16_Data[aps_State->u8_Index]);
        aps_State->u8_Index++;
      } else {
        aps_State->e_MainState = GATEDRIVER_DRV8323_READ_CONFIGURATION_REGISTERS;
        aps_State->u8_Index    = 0U;
      }
      break;
    case GATEDRIVER_DRV8323_READ_CONFIGURATION_REGISTERS:
      if (aps_State->u8_Index < GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE) {
        /* read using spi */
        aps_State->s_Configuration.au16_Data[aps_State->u8_Index] =
            ReadAddress(ConfigurationRegisters[aps_State->u8_Index]);
        aps_State->u8_Index++;
      } else {
        for (lu16_Index = 0U; lu16_Index < GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE; lu16_Index++) {
          if (aps_State->s_Configuration.au16_Data[lu16_Index] !=
              Configuration.au16_Data[lu16_Index]) {
            /* actual does not match desired config */
            break;
          }
        }
        if (lu16_Index == GATEDRIVER_DRV8323_CONFIG_REGISTER_SIZE) {
          aps_State->e_MainState                          = GATEDRIVER_DRV8323_READ_STATUS_REGISTERS;
          aps_State->u8_Index                             = 0U;
          aps_State->s_Error.u8_DriverIsUnreachableViaSpi = 0U;
        } else {
          Hal_Spi_Close();
          mpwm_DisableGDU();
          aps_State->s_Error.u8_DriverIsUnreachableViaSpi = 1U;
          aps_State->e_MainState                          = GATEDRIVER_DRV8323_FAILED;
        }
      }
      break;
    case GATEDRIVER_DRV8323_READ_STATUS_REGISTERS:
      if (aps_State->u8_Index < GATEDRIVER_DRV8323_ERROR_REGISTER_SIZE) {
        /* read using spi */
        aps_State->s_Status.au16_Data[aps_State->u8_Index] =
            ReadAddress(StatusRegisters[aps_State->u8_Index]);
        aps_State->u8_Index++;
      } else {
        Hal_Spi_Close();
        aps_State->e_MainState = GATEDRIVER_DRV8323_DONE;
        aps_State->u8_Index    = 0U;
      }
      break;
    case GATEDRIVER_DRV8323_DONE:
    case GATEDRIVER_DRV8323_FAILED:
      break;
  }
}


void GatedriverDrv8323_GetErrors(const GatedriverDrv8323StateT *aps_State,
                                 GatedriverDrv8323ErrorT *aps_Errors) {
  aps_Errors->s_CommonErrors.u8_UnderVoltagePresent    = aps_State->s_Status.s_Register.s_FSR1.u16_UVLO;
  aps_Errors->s_CommonErrors.u8_OverVoltagePresent     = 0U;
  if ((aps_State->s_Status.s_Register.s_FSR2.u16_SA_OC != 0U) ||
      (aps_State->s_Status.s_Register.s_FSR2.u16_SB_OC != 0U) ||
      (aps_State->s_Status.s_Register.s_FSR2.u16_SC_OC != 0U)) {
    aps_Errors->s_CommonErrors.u8_OverCurrentPresent = 1U;
  } else {
    aps_Errors->s_CommonErrors.u8_OverCurrentPresent = 0U;
  }
  aps_Errors->s_CommonErrors.u8_DesatPresent           = aps_State->s_Status.s_Register.s_FSR1.u16_VDS_OCP;
  aps_Errors->s_CommonErrors.u8_InternalError          = aps_State->s_Status.s_Register.s_FSR1.u16_GDF;
  aps_Errors->s_CommonErrors.u8_OverTemperaturePresent = aps_State->s_Status.s_Register.s_FSR1.u16_OTSD;
}





/* end of file */
