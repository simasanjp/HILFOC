/*
 * TerminalHandler.c
 *
 *  Created on: Oct 14, 2020
 *      Author: gdobrev
 */

#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "netx_drv.h"
#include "FOC_APPL.h"
#include "TerminalHandler.h"

/*****************************************************************************/
/*! DEFINITIONS                                                              */
/*****************************************************************************/

#define APP_TERMINAL_MAX_CMDLINE_LEN   64 /* Terminal buffer size */
#define APP_TERMINAL_MAX_CMD_LEN       20 /* Terminal command buffer size */
extern DRV_UART_HANDLE_T g_tUART;      /* global Uart handle defined in libc_file_support.c */

/*****************************************************************************/
/*! ENUMERATIONS of terminal commands                                        */
/*****************************************************************************/
typedef enum APP_TERMINAL_HANDLER_COMMANDS_Etag
{
  APP_TERMINAL_HANDLER_COMMAND_MOTOR_START,
  APP_TERMINAL_HANDLER_COMMAND_MOTOR_GET_SPEED,
  APP_TERMINAL_HANDLER_COMMAND_MOTOR_STOP,

  APP_TERMINAL_HANDLER_NUMBER_OF_COMMANDS,
  APP_TERMINAL_HANDLER_COMMAND_UNKNOWN,
} APP_TERMINAL_HANDLER_COMMANDS_E;

/*****************************************************************************/
/*! typedef terminal command structure                                       */
/*****************************************************************************/
typedef struct APP_TERMINAL_CMD_Ttag
{
  uint8_t bCommandCode;
  const char    *szString;
  const char    *szHelp;
} APP_TERMINAL_CMD_T;


/*****************************************************************************/
/*! FUNCTION PROTOTYPES                                                      */
/*****************************************************************************/
int kbhit(void);

/** Terminal service commands */
APP_TERMINAL_CMD_T g_tTerminalCommandsDictionary[] =
{
  {APP_TERMINAL_HANDLER_COMMAND_MOTOR_START,              "motor_start",                 "Starts the motor"         },
  {APP_TERMINAL_HANDLER_COMMAND_MOTOR_GET_SPEED,          "motor_get_speed",             "Get current speed in RPM" },
  {APP_TERMINAL_HANDLER_COMMAND_MOTOR_STOP,               "motor_stop",                  "Stops the motor"          },
};

/*****************************************************************************/
/*! FUNCTIONS                                                                */
/*****************************************************************************/

/*****************************************************************************/
/*! Translates the given command name to the appropriate command code
 *
 *   \param szCmd [in] Command name
 *
 *   \return Command code
 */
/*****************************************************************************/
static uint8_t Term_GetCommandCodeByCommandName(char* szCommandName)
{
  uint8_t bIndex = 0;

  for(bIndex = 0; bIndex < APP_TERMINAL_HANDLER_NUMBER_OF_COMMANDS; bIndex++)
  {
    if(0 == strcmp(szCommandName, &g_tTerminalCommandsDictionary[bIndex].szString[0]))
    {
      return g_tTerminalCommandsDictionary[bIndex].bCommandCode;
    }
  }

  return APP_TERMINAL_HANDLER_COMMAND_UNKNOWN;
}

/*****************************************************************************/
/*! Handler for incoming commands
 *
 *   \param ptAppData             [in] pointer to APP_DATA_T structure
 *   \param uiTerminalCommandCode [in] terminal command code
 *   \param iNumberOfArguments    [in] Number of arguments in auiArgumentList
 *   \param auiArgumentList       [in] Argument list
 */
/*****************************************************************************/
static void Term_TerminalCommandHandler(APP_DATA_T * ptData,
                                        int           uiTerminalCommandCode,
                                        int           iNumberOfArguments,
                                        unsigned int* auiArgumentList)
{

  switch(uiTerminalCommandCode)
  {
    case APP_TERMINAL_HANDLER_COMMAND_MOTOR_START:
    {
      printf("Received APP_TERMINAL_HANDLER_COMMAND_MOTOR_START\r\n");
      FOC_APPL_setSpeedSetpoint(DEMO_MOTOR_START_SPEED, ptData->tFOC_APPLData);
      break;
    }

    case APP_TERMINAL_HANDLER_COMMAND_MOTOR_GET_SPEED:
    {
      printf("Received APP_TERMINAL_HANDLER_COMMAND_MOTOR_GET_SPEED\r\n");
      printf("Current speed is %d RPM\r\n", FOC_APPL_getSpeed(ptData->tFOC_APPLData));
      break;
    }

    case APP_TERMINAL_HANDLER_COMMAND_MOTOR_STOP:
    {
      printf("Received APP_TERMINAL_HANDLER_COMMAND_MOTOR_STOP\r\n");
      FOC_APPL_setSpeedSetpoint(DEMO_MOTOR_STOP_SPEED, ptData->tFOC_APPLData);
      break;
    }

    case APP_TERMINAL_HANDLER_COMMAND_UNKNOWN:
    default:
    {
      printf("Received APP_TERMINAL_HANDLER_COMMAND_UNKNOWN\r\n"
             "Possible commands:\r\n"
             "1. motor_start\r\n"
             "2. motor_get_speed\r\n"
             "3. motor_stop\r\n");
      break;
    }
  }
}


#define STR(tok) #tok
#define XSTR(tok) STR(tok)

/*****************************************************************************/
/*! Entry point for terminal handle. Needs to be called cyclically in order to
 *  process incoming terminal commands.
 *
 *   \param ptAppData [in]  Pointer to application data
 */
/*****************************************************************************/
void App_TerminalHandler(APP_DATA_T * ptData)
{
  static char   s_szInBuffer[APP_TERMINAL_MAX_CMDLINE_LEN];
  static uint_fast16_t    s_iKeyCnt = 0;
  uint_fast8_t           iKey      = 0;
  int           iNumberOfArguments     = 0;
  char          szCmd[APP_TERMINAL_MAX_CMD_LEN + 1];   /** plus one for the terminating NUL character */
  unsigned int  auiArgumentList[3];

  if(kbhit())
  {
    iKey = getchar();

    if(iKey == '\r' || iKey == '\n')
    {
      s_szInBuffer[s_iKeyCnt] = '\0';
      s_iKeyCnt               = 0;
      iNumberOfArguments = sscanf( &s_szInBuffer[0],
                                   "%" XSTR(APP_TERMINAL_MAX_CMD_LEN) "s %x %x %x",
                                   &szCmd[0],
                                   &auiArgumentList[0],
                                   &auiArgumentList[1],
                                   &auiArgumentList[2] ); /** command length is hardcoded here*/


      int iTerminalCommandCode = Term_GetCommandCodeByCommandName(&szCmd[0]);
      Term_TerminalCommandHandler(ptData,
                                  iTerminalCommandCode,
                                  iNumberOfArguments,
                                  &auiArgumentList[0]);
    }
    else
    {
      s_szInBuffer[s_iKeyCnt] = (char)iKey;
      s_iKeyCnt++;

      if(s_iKeyCnt >= APP_TERMINAL_MAX_CMDLINE_LEN)
      {
        s_iKeyCnt = 0;
      }
    }
  }
}

/*****************************************************************************/
/*! Tests if any data in Terminal Uart FIFO
 *   \return   0 -- FIFO is empty
 1 -- data in FIFO                                              */
/*****************************************************************************/
int kbhit(void)
{
  DRV_UART_STATE_E eState;

  while(DRV_OK != DRV_UART_GetState(&g_tUART, &eState))
    ;

  if((eState & DRV_UART_STATE_RXFE) != 0) /* check if there is data in the FIFO  */
    return 0; /* no, FIFO is empty  */
  else
    return 1; /* yes, data in FIFO  */
}
