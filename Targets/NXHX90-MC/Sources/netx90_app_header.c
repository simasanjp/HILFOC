/**************************************************************************//**
 * @file    netx90_app_header_useCaseA.c
 * @brief   netx90 app header for use case A
 * $Revision: 7941 $
 * $Date: 2020-06-23 11:24:12 +0200 (Di, 23 Jun 2020) $
 ******************************************************************************/

/**************************************************************************************
 Exclusion of Liability for this demo software:
 The following software is intended for and must only be used for reference and in an
 evaluation laboratory environment. It is provided without charge and is subject to
 alterations. There is no warranty for the software, to the extent permitted by
 applicable law. Except when otherwise stated in writing the copyright holders and/or
 other parties provide the software "as is" without warranty of any kind, either
 expressed or implied.
 Please refer to the Agreement in README_DISCLAIMER.txt, provided together with this file!
 By installing or otherwise using the software, you accept the terms of this Agreement.
 If you do not agree to the terms of this Agreement, then do not install or use the
 Software!
 **************************************************************************************/

/**************************************************************************************

 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.


 **************************************************************************************/
#include <stdio.h>

#include "netx_drv.h"
#include "Hil_FileHeaderV3.h"
//#include "Hil_SharedDefines.h"

NETX_CPU_BOOTBLOCK_T app_cpu_hboot_header_nai __attribute__ ((section (".app_cpu_hboot_boot_header_nai"))) =
{
  .ulMagic = NETX_CPU_BOOTBLOCK_MAGIC,
  .ulFlashOffsetBytes = 0,
  .pulNextHeader = 0,
  .pulDestination = 0,
  .ulImageSizeDword = 0,
  .ulFlashSelection = 0,
  .ulSignature = NETX_CPU_BOOTBLOCK_SIGNATURE,
  .ulReserved0 = 0,
  .aulHash = { 0 },
  .ulBootHeaderChecksum = 0,
};

NETX_CPU_BOOTBLOCK_T app_cpu_hboot_header_nae __attribute__ ((section (".app_cpu_hboot_boot_header_nae"))) =
{
  .ulMagic = NETX_CPU_BOOTBLOCK_MAGIC,
  .ulFlashOffsetBytes = 0,
  .pulNextHeader = 0,
  .pulDestination = 0,
  .ulImageSizeDword = 0,
  .ulFlashSelection = 0,
  .ulSignature = NETX_CPU_BOOTBLOCK_SIGNATURE,
  .ulReserved0 = 0,
  .aulHash = { 0 },
  .ulBootHeaderChecksum = 0,
};

void print_app_hboot_header(NETX_CPU_BOOTBLOCK_T* ptHeader)
{
  printf("\nprint_app_hboot_header\n");
  printf("  Pointer: 0x%08x\n", (uintptr_t) ptHeader);
  printf("  .ulMagic               = 0x%08x\n", (unsigned int) ptHeader->ulMagic);
  printf("  .ulFlashOffsetBytes    = 0x%08x\n", (unsigned int) ptHeader->ulFlashOffsetBytes);
  printf("  .pulNextHeader         = 0x%08x\n", (unsigned int) ptHeader->pulNextHeader);
  printf("  .pulDestination        = 0x%08x\n", (unsigned int) ptHeader->pulDestination);
  printf("  .ulImageSizeDword      = 0x%08x\n", (unsigned int) ptHeader->ulImageSizeDword);
  printf("  .ulFlashSelection      = 0x%08x\n", (unsigned int) ptHeader->ulFlashSelection);
  printf("  .ulSignature           = 0x%08x\n", (unsigned int) ptHeader->ulSignature);
  printf("  .aulHash               = 0x%08x\n", (unsigned int) ptHeader->aulHash[0]);
  printf("  .ulBootHeaderChecksum  = 0x%08x\n", (unsigned int) ptHeader->ulBootHeaderChecksum);
}

HIL_FILE_NAI_HEADER_V3_0_T app_cpu_header_nai __attribute__ ((section (".app_cpu_header_nai"))) =
{
  .tBootHeader =
  {
    .ulMagicCookie = HIL_FILE_HEADER_FIRMWARE_NAI_COOKIE,
    .ulAppChecksum = 0,
    .ulAppFileSize = 0,
    .ulSignature = HIL_FILE_BOOT_HEADER_SIGNATURE,
    .ulBootHeaderChecksum = 0,
  },
  .tCommonHeader =
  {
    .ulHeaderVersion = HIL_VERSION_COMMON_HEADER_3_0,
    .ulHeaderLength = sizeof(app_cpu_header_nai),
    .ulDataSize = 0,
    .ulDataStartOffset = 704,
    .bNumModuleInfos = 0,
    .aulMD5 = {0},
    .ulTagListSize = 0,
    .ulTagListOffset = 0,
    .ulTagListSizeMax = 0,
    .ulCommonCRC32 = 0,
    .ulHeaderCRC32 = 0,
  },
  .tDeviceInfo =
  {
    .ulStructVersion = HIL_VERSION_DEVICE_INFO_V1_0,
    .usManufacturer = HIL_MANUFACTURER_HILSCHER_GMBH,
    .usDeviceClass = HIL_HW_DEV_CLASS_CHIP_NETX_90_APP_FOR_COM_USECASE_A,
    .bHwCompatibility = 0,
    .bChipType = HIL_DEV_CHIP_TYPE_NETX90,
    .ausHwOptions = {HIL_HW_ASSEMBLY_ETHERNET, HIL_HW_ASSEMBLY_ETHERNET, HIL_HW_ASSEMBLY_UNDEFINED, HIL_HW_ASSEMBLY_UNDEFINED},
    .ulLicenseFlags1 = 0,
    .ulLicenseFlags2 = 0,
    .usNetXLicenseID = 0,
    .usNetXLicenseFlags = 0,
    .ausFwVersion = {0},
    .ulDeviceNumber = 0,
    .ulFwNumber = 0,
    .ulSerialNumber = 0,
  },
};


HIL_FILE_NAI_HEADER_V3_0_T app_cpu_header_nae __attribute__ ((section (".app_cpu_header_nae"))) =
{
  .tBootHeader =
  {
    .ulMagicCookie = HIL_FILE_HEADER_FIRMWARE_NAE_COOKIE,
    .ulAppChecksum = 0,
    .ulAppFileSize = 0,
    .ulSignature = HIL_FILE_BOOT_HEADER_SIGNATURE,
    .ulBootHeaderChecksum = 0,
  },
  .tCommonHeader =
  {
    .ulHeaderVersion = HIL_VERSION_COMMON_HEADER_3_0,
    .ulHeaderLength = sizeof(app_cpu_header_nae),
    .ulDataSize = 0,
    .ulDataStartOffset = 0,
    .bNumModuleInfos = 0,
    .aulMD5 = {0},
    .ulTagListSize = 0,
    .ulTagListOffset = 0,
    .ulTagListSizeMax = 0,
    .ulCommonCRC32 = 0,
    .ulHeaderCRC32 = 0,
  },
  .tDeviceInfo =
  {
    .ulStructVersion = HIL_VERSION_DEVICE_INFO_V1_0,
    .usManufacturer = HIL_MANUFACTURER_HILSCHER_GMBH,
    .usDeviceClass = HIL_HW_DEV_CLASS_CHIP_NETX_90_APP_FOR_COM_USECASE_A,
    .bHwCompatibility = 0,
    .bChipType = HIL_DEV_CHIP_TYPE_NETX90,
    .ausHwOptions = {HIL_HW_ASSEMBLY_ETHERNET, HIL_HW_ASSEMBLY_ETHERNET, HIL_HW_ASSEMBLY_UNDEFINED, HIL_HW_ASSEMBLY_UNDEFINED},
    .ulLicenseFlags1 = 0,
    .ulLicenseFlags2 = 0,
    .usNetXLicenseID = 0,
    .usNetXLicenseFlags = 0,
    .ausFwVersion = {0},
    .ulDeviceNumber = 0,
    .ulFwNumber = 0,
    .ulSerialNumber = 0,
  },
};
