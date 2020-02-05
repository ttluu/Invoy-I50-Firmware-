/**
 * @brief       bme680_defs.h
 * @details     Low power gas, pressure, temperature & humidity sensor.
 *              Header file.
 *
 *
 * @return      N/A
 *
 * @author      Manuel Caballero
 * @date        21/July/2018
 * @version     21/July/2018    The ORIGIN
 * @pre         This is just a port from Bosh driver to mBed ( c++ )
 * @warning     N/A
 * @pre         This code belongs to Nimbus Centre ( http://www.nimbus.cit.ie ).
 */
 /**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bme680_defs.h
 * @date    19 Jun 2018
 * @version 3.5.9
 * @brief
 *
 */
 
/*! @file bme680_defs.h
 @brief Sensor driver for BME680 sensor */
/*!
 * @defgroup BME680 SENSOR API
 * @brief
 * @{*/
#ifndef BME680_DEFS_H_
#define BME680_DEFS_H_
 
/********************************************************/
/* header includes */
//#ifdef __KERNEL__
//#include <linux/types.h>
//#include <linux/kernel.h>
//#else
#include <stdint.h>
#include <stddef.h>
//#endif
 
/******************************************************************************/
/*! @name       Common macros                         */
/******************************************************************************/
 
 
#include <.\Drivers\stdlib.h>
#include <.\Drivers\math.h>
#include <.\Drivers\stdio.h>
#include <.\Drivers\string.h>
#include <.\Drivers\ieeefloat.c>

////////////////////////////////////////

//#define STREAM_SERIAL_INPUT STDOUT
//#include <.\Drivers\input.c>

////////////////////////////////////////
/*
typedef int8 int8_t;
typedef int16 int16_t;
typedef int32 int32_t;
typedef long long int64_t;
typedef unsigned int8 uint8_t;
typedef unsigned int16 uint16_t;
typedef unsigned int32 uint32_t;
typedef unsigned long long uint64_t;
*/

/*
typedef (int8_t) (int8);
typedef (int16) INT16_C;
typedef (int32) (int32);
typedef (unsigned int8) (unsigned int8);
typedef (unsigned int16) (unsigned int16);
typedef (unsigned int32) U(int32);
*/
 
/*
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       int8_t(x)
#define UINT8_C(x)      uint8_t(x)
#endif
 
#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      int16_t(x)
#define UINT16_C(x)     uint16_t(x)
#endif
 
#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      int32_t(x)
#define UINT32_C(x)     uint32_t(x)
#endif
 
#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      int64_t(x)
#define UINT64_C(x)     uint64_t(x)
#endif
 */
 
/**@}*/
 
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif
 
/** BME680 configuration macros */
/** Enable or un-comment the macro to provide floating point data output */
#ifndef BME680_FLOAT_POINT_COMPENSATION
// #define BME680_FLOAT_POINT_COMPENSATION */
#endif
 
/** BME680 General config */
#define BME680_POLL_PERIOD_MS             (unsigned int8)(10)
 
/** BME680 I2C addresses */
//  T. Luu Check this address shifting compatibility!!!!!
#define BME680_I2C_ADDR_PRIMARY           (unsigned int8)( 0x76 )        // [MANUAL] Address compatible with mBed
#define BME680_I2C_ADDR_SECONDARY         (unsigned int8)( 0x77 )        // [MANUAL] Address compatible with mBed
 
#define BME680_I2C_Read_addr            ((BME680_I2C_ADDR_PRIMARY << 1) | 0x01)
#define BME680_I2C_Write_addr           ((BME680_I2C_ADDR_PRIMARY << 1) & 0xFE)

/** BME680 unique chip identifier */
#define BME680_CHIP_ID                    (unsigned int8)(0x61)


/** BME680 coefficients related defines */
#define BME680_COEFF_SIZE                 (unsigned int8)(41)
#define BME680_COEFF_ADDR1_LEN            (unsigned int8)(25)
#define BME680_COEFF_ADDR2_LEN            (unsigned int8)(16)
 
/** BME680 field_x related defines */
#define BME680_FIELD_LENGTH               (unsigned int8)(15)
#define BME680_FIELD_ADDR_OFFSET          (unsigned int8)(17)
 
/** Soft reset command */
#define BME680_SOFT_RESET_CMD             (unsigned int8)(0xb6)
 
/** Error code definitions */
#define BME680_OK                         (unsigned int8)(0)//(int8)(0)

/* Errors */
#define BME680_E_NULL_PTR                 (unsigned int8)(1)
#define BME680_E_COM_FAIL                 (unsigned int8)(2)
#define BME680_E_DEV_NOT_FOUND            (unsigned int8)(4)
#define BME680_E_INVALID_LENGTH           (unsigned int8)(8)
#define BME680_E_SET_MEM_PAGE_FAIL        (unsigned int8)(16)
#define BME680_E_XXXX_FAIL                (unsigned int8)(32)
#define BME680_E_XXXX1_FAIL               (unsigned int8)(64)
#define BME680_E_XXXX2_FAIL               (unsigned int8)(128)
 
/* Warnings */
#define BME680_W_DEFINE_PWR_MODE          (int8)(1)
#define BME680_W_NO_NEW_DATA              (int8)(2)
 
/* Info's */
#define BME680_I_MIN_CORRECTION           (unsigned int8)(1)
#define BME680_I_MAX_CORRECTION           (unsigned int8)(2)
 
/** Register map */
/** Other coefficient's address */
#define BME680_ADDR_RES_HEAT_VAL_ADDR     (unsigned int8)(0x00)
#define BME680_ADDR_RES_HEAT_RANGE_ADDR   (unsigned int8)(0x02)
#define BME680_ADDR_RANGE_SW_ERR_ADDR     (unsigned int8)(0x04)
#define BME680_ADDR_SENS_CONF_START       (unsigned int8)(0x5A)
#define BME680_ADDR_GAS_CONF_START        (unsigned int8)(0x64)
 
/** Field settings */
#define BME680_FIELD0_ADDR                (unsigned int8)(0x1d)
 
/** Heater settings */
#define BME680_RES_HEAT0_ADDR             (unsigned int8)(0x5a)
#define BME680_GAS_WAIT0_ADDR             (unsigned int8)(0x64)
 
/** Sensor configuration registers */
#define BME680_CONF_HEAT_CTRL_ADDR        (unsigned int8)(0x70)
#define BME680_CONF_ODR_RUN_GAS_NBC_ADDR  (unsigned int8)(0x71)
#define BME680_CONF_OS_H_ADDR             (unsigned int8)(0x72)
#define BME680_MEM_PAGE_ADDR              (unsigned int8)(0x73)
#define BME680_CONF_T_P_MODE_ADDR         (unsigned int8)(0x74)
#define BME680_CONF_ODR_FILT_ADDR         (unsigned int8)(0x75)
 
/** Coefficient's address */
#define BME680_COEFF_ADDR1                (unsigned int8)(0x89)
#define BME680_COEFF_ADDR2                (unsigned int8)(0xe1)
 
/** Chip identifier */
#define BME680_I2CCHIP_ID_ADDR            (unsigned int8)(0xd0)
#define BME680_SPI_CHIP_ID_ADDR           (unsigned int8)(0x50) 

/** Soft reset register */
#define BME680_I2C_SOFT_RESET_ADDR        (unsigned int8)(0xe0)
#define BME680_SPI_SOFT_RESET_ADDR        (unsigned int8)(0x60) 

/** Heater control settings */
#define BME680_ENABLE_HEATER              (unsigned int8)(0x00)
#define BME680_DISABLE_HEATER             (unsigned int8)(0x08)
 
/** Gas measurement settings */
#define BME680_DISABLE_GAS_MEAS           (unsigned int8)(0x00)
#define BME680_ENABLE_GAS_MEAS            (unsigned int8)(0x01)
 
/** Over-sampling settings */
#define BME680_OS_NONE                    (unsigned int8)(0)
#define BME680_OS_1X                      (unsigned int8)(1)
#define BME680_OS_2X                      (unsigned int8)(2)
#define BME680_OS_4X                      (unsigned int8)(3)
#define BME680_OS_8X                      (unsigned int8)(4)
#define BME680_OS_16X                     (unsigned int8)(5)
 
/** IIR filter settings */
#define BME680_FILTER_SIZE_0              (unsigned int8)(0)
#define BME680_FILTER_SIZE_1              (unsigned int8)(1)
#define BME680_FILTER_SIZE_3              (unsigned int8)(2)
#define BME680_FILTER_SIZE_7              (unsigned int8)(3)
#define BME680_FILTER_SIZE_15             (unsigned int8)(4)
#define BME680_FILTER_SIZE_31             (unsigned int8)(5)
#define BME680_FILTER_SIZE_63             (unsigned int8)(6)
#define BME680_FILTER_SIZE_127            (unsigned int8)(7)
 
/** Power mode settings */
#define BME680_SLEEP_MODE                 (unsigned int8)(0)
#define BME680_FORCED_MODE                (unsigned int8)(1)
 
/** Delay related macro declaration */
//#define BME680_RESET_PERIOD               (unsigned int32)(int32)(10)
#define BME680_RESET_PERIOD               (unsigned int32)(10) 

/** SPI memory page settings */
#define BME680_MEM_PAGE0                  (unsigned int8)(0x0)
#define BME680_MEM_PAGE1                  (unsigned int8)(0x2)

/** SPI register settings */
#define BME680_SPI_STATUS                (unsigned int8)(0x73)
#define BME680_SPI_RESET                 (unsigned int8)(0x60)
#define BME680_SPI_DEVICE_ID             (unsigned int8)(0x50)
#define BME680_SPI_CONFIG                (unsigned int8)(0x75)
#define BME680_SPI_CTRL                  (unsigned int8)(0x74)
#define BME680_SPI_CTRL_HUM              (unsigned int8)(0x72)
#define BME680_SPI_CTRL_GAS_1            (unsigned int8)(0x71)
#define BME680_SPI_CTRL_GAS_0            (unsigned int8)(0x70)
#define BME680_SPI_GAS_WAIT              (unsigned int8)(0x64) 
#define BME680_SPI_RES_HEAT_X            (unsigned int8)(0x5a)
#define BME680_SPI_IDAC_HEAT_X           (unsigned int8)(0x50)
#define BME680_SPI_GAS_R_LSB             (unsigned int8)(0x2b)
#define BME680_SPI_GAS_R_MSB             (unsigned int8)(0x2a) 
#define BME680_SPI_HUM_LSB               (unsigned int8)(0x26)
#define BME680_SPI_HUM_MSB               (unsigned int8)(0x25)
#define BME680_SPI_TEMP_XLSB             (unsigned int8)(0x24)
#define BME680_SPI_TEMP_LSB              (unsigned int8)(0x23)
#define BME680_SPI_TEMP_MSB              (unsigned int8)(0x22)
#define BME680_SPI_PRESS_XLSB            (unsigned int8)(0x21)
#define BME680_SPI_PRESS_LSB             (unsigned int8)(0x20)
#define BME680_SPI_PRESS_MSB             (unsigned int8)(0x1f)
#define BME680_SPI_EAS_STATUS_0          (unsigned int8)(0x1d)
 
#define SLEEP_MODE      (unsigned int8)0X0
#define FORCED_MODE     (unsigned int8)0X1

#define PORTC           (unsigned int8)0XF82
#define LATC            (unsigned int8)0xF8B
#define ANSELC          (unsigned int8)0xF3A
#define TRISC           (unsigned int8)0xF94
#define SSP1CON1        (unsigned int8)0XFC6
#define SSP1STAT        (unsigned int8)0XFC7

#define TMP_HUM_ADDR    (unsigned int8)0X25
#define TMP_MSB_ADDR    (unsigned int8)0X22
#define PRESS_MSB_ADDR  (unsigned int8)0X1F
#define IDAX_HEAT_X     (unsigned int8)0X50
#define RES_WAIT_X      (unsigned int8)0X5A
#define GAS_WAIT_X      (unsigned int8)0X64
#define CTRL_GAS_1      (unsigned int8)0X71

#define BME680_W_SELF_TEST_FAILED 3

#define MIN_TEMPERATURE (uint16)(0)      /* 0 degree Celsius */
#define MAX_TEMPERATURE (uint16)(6000)    /* 60 degree Celsius */

#define MIN_PRESSURE (uint32)(90000)   /* 900 hecto Pascals */
#define MAX_PRESSURE (uint32)(110000)    /* 1100 hecto Pascals */

#define MIN_HUMIDITY (uint32)(20000)   /* 20% relative humidity */
#define MAX_HUMIDITY (uint32)(80000)    /* 80% relative humidity*/

#define HEATR_DUR          2000
#define N_MEAS             6
#define LOW_TEMP           150
#define HIGH_TEMP          350
 
 
/** Ambient humidity shift value for compensation */
#define BME680_HUM_REG_SHIFT_VAL          (unsigned int8)(4)
 
/** Run gas enable and disable settings */
#define BME680_RUN_GAS_DISABLE            (unsigned int8)(0)
#define BME680_RUN_GAS_ENABLE             (unsigned int8)(1)
 
/** Buffer length macro declaration */
#define BME680_TMP_BUFFER_LENGTH          (unsigned int8)(40)
#define BME680_REG_BUFFER_LENGTH          (unsigned int8)(6)
#define BME680_FIELD_DATA_LENGTH          (unsigned int8)(3)
#define BME680_GAS_REG_BUF_LENGTH         (unsigned int8)(20)
 
/** Settings selector */
#define BME680_OST_SEL                    (unsigned int16)(1)
#define BME680_OSP_SEL                    (unsigned int16)(2)
#define BME680_OSH_SEL                    (unsigned int16)(4)
#define BME680_GAS_MEAS_SEL               (unsigned int16)(8)
#define BME680_FILTER_SEL                 (unsigned int16)(16)
#define BME680_HCNTRL_SEL                 (unsigned int16)(32)
#define BME680_RUN_GAS_SEL                (unsigned int16)(64)
#define BME680_NBCONV_SEL                 (unsigned int16)(128)
#define BME680_GAS_SENSOR_SEL             (BME680_GAS_MEAS_SEL | BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)
 
/** Number of conversion settings*/
#define BME680_NBCONV_MIN        (unsigned int8)(0)
#define BME680_NBCONV_MAX        (unsigned int8)(10)
 
/** Mask definitions */
#define BME680_GAS_MEAS_MSK      (unsigned int8)(0x30)
#define BME680_NBCONV_MSK        (unsigned int8)(0X0F)
#define BME680_FILTER_MSK        (unsigned int8)(0X1C)
#define BME680_OST_MSK           (unsigned int8)(0XE0)
#define BME680_OSP_MSK           (unsigned int8)(0X1C)
#define BME680_OSH_MSK           (unsigned int8)(0X07)
#define BME680_HCTRL_MSK         (unsigned int8)(0x08)
#define BME680_RUN_GAS_MSK       (unsigned int8)(0x10)
#define BME680_MODE_MSK          (unsigned int8)(0x03)
#define BME680_RHRANGE_MSK       (unsigned int8)(0x30)
#define BME680_RSERROR_MSK       (unsigned int8)(0xf0)
#define BME680_NEW_DATA_MSK      (unsigned int8)(0x80)
#define BME680_GAS_INDEX_MSK     (unsigned int8)(0x0f)
#define BME680_GAS_RANGE_MSK     (unsigned int8)(0x0f)
#define BME680_GASM_VALID_MSK    (unsigned int8)(0x20)
#define BME680_HEAT_STAB_MSK     (unsigned int8)(0x10)
#define BME680_MEM_PAGE_MSK      (unsigned int8)(0x10)
#define BME680_SPI_RD_MSK        (unsigned int8)(0x80)
#define BME680_SPI_WR_MSK        (unsigned int8)(0x7f)
#define BME680_BIT_H1_DATA_MSK   (unsigned int8)(0x0F)
 
/** Bit position definitions for sensor settings */
#define BME680_GAS_MEAS_POS      (unsigned int8)(4)
#define BME680_FILTER_POS        (unsigned int8)(2)
#define BME680_OST_POS           (unsigned int8)(5)
#define BME680_OSP_POS           (unsigned int8)(2)
#define BME680_RUN_GAS_POS       (unsigned int8)(4)
 
/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG   (1)
#define BME680_T2_MSB_REG   (2)
#define BME680_T3_REG       (3)
#define BME680_P1_LSB_REG   (5)
#define BME680_P1_MSB_REG   (6)
#define BME680_P2_LSB_REG   (7)
#define BME680_P2_MSB_REG   (8)
#define BME680_P3_REG       (9)
#define BME680_P4_LSB_REG   (11)
#define BME680_P4_MSB_REG   (12)
#define BME680_P5_LSB_REG   (13)
#define BME680_P5_MSB_REG   (14)
#define BME680_P7_REG       (15)
#define BME680_P6_REG       (16)
#define BME680_P8_LSB_REG   (19)
#define BME680_P8_MSB_REG   (20)
#define BME680_P9_LSB_REG   (21)
#define BME680_P9_MSB_REG   (22)
#define BME680_P10_REG      (23)
#define BME680_H2_MSB_REG   (25)
#define BME680_H2_LSB_REG   (26)
#define BME680_H1_LSB_REG   (26)
#define BME680_H1_MSB_REG   (27)
#define BME680_H3_REG       (28)
#define BME680_H4_REG       (29)
#define BME680_H5_REG       (30)
#define BME680_H6_REG       (31)
#define BME680_H7_REG       (32)
#define BME680_T1_LSB_REG   (33)
#define BME680_T1_MSB_REG   (34)
#define BME680_GH2_LSB_REG  (35)
#define BME680_GH2_MSB_REG  (36)
#define BME680_GH1_REG      (37)
#define BME680_GH3_REG      (38)
 
/** BME680 register buffer index settings*/
#define BME680_REG_FILTER_INDEX        (unsigned int8)(5)
#define BME680_REG_TEMP_INDEX          (unsigned int8)(4)
#define BME680_REG_PRES_INDEX          (unsigned int8)(4)
#define BME680_REG_HUM_INDEX           (unsigned int8)(2)
#define BME680_REG_NBCONV_INDEX        (unsigned int8)(1)
#define BME680_REG_RUN_GAS_INDEX       (unsigned int8)(1)
#define BME680_REG_HCTRL_INDEX         (unsigned int8)(0)
 
/** BME680 pressure calculation macros */
/*! This max value is used to provide precedence to multiplication or division
 * in pressure compensation equation to achieve least loss of precision and
 * avoiding overflows.
 * i.e Comparing value, BME680_MAX_OVERFLOW_VAL = (int32)(1 << 30)
 */
#define BME680_MAX_OVERFLOW_VAL      (int32)(0x40000000)
 
/** Macro to combine two 8 bit data's to form a 16 bit data */
#define BME680_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)
 
/** Macro to SET and GET BITS of a register */
#define BME680_SET_BITS(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        ((data << bitname##_POS) & bitname##_MSK))
#define BME680_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
    (bitname##_POS))
 
/** Macro variant to handle the bitname position if it is zero */
#define BME680_SET_BITS_POS_0(reg_data, bitname, data) \
                ((reg_data & ~(bitname##_MSK)) | \
                (data & bitname##_MSK))
#define BME680_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))
 
/** Type definitions */
/*!
 * Generic communication function pointer
 * @param[in] dev_id: Place holder to store the id of the device structure
 *                    Can be used to store the index of the Chip select or
 *                    I2C address of the device.
 * @param[in] reg_addr: Used to select the register the where data needs to
 *                      be read from or written to.
 * @param[in/out] reg_data: Data array to read/write
 * @param[in] len: Length of the data array
 */
typedef int8_t (*bme680_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
 
/*!
 * Delay function pointer
 * @param[in] period: Time period in milliseconds
 */
typedef void (*bme680_delay_fptr_t)(uint32_t period);
 
/*!
 * @brief Interface selection Enumerations
 */
enum bme680_intf {
    /*! SPI interface */
    BME680_SPI_INTF,
    /*! I2C interface */
    BME680_I2C_INTF
};
 
/* structure definitions */
/*!
 * @brief Sensor field data structure
 */
struct  bme680_field_data {
    /*! Contains new_data, gasm_valid & heat_stab */
    uint8_t status;
    /*! The index of the heater profile used */
    uint8_t gas_index;
    /*! Measurement index to track order */
    uint8_t meas_index;
 
#ifndef BME680_FLOAT_POINT_COMPENSATION
    /*! Temperature in degree celsius x100 */
    int16_t temperature;
    /*! Pressure in Pascal */
    uint32_t pressure;
    /*! Humidity in % relative humidity x1000 */
    uint32_t humidity;
    /*! Gas resistance in Ohms */
    uint32_t gas_resistance;
#else
    /*! Temperature in degree celsius */
    float temperature;
    /*! Pressure in Pascal */
    float pressure;
    /*! Humidity in % relative humidity x1000 */
    float humidity;
    /*! Gas resistance in Ohms */
    float gas_resistance;
 
#endif
 
};
 
/*!
 * @brief Structure to hold the Calibration data
 */
struct  bme680_calib_data {
    /*! Variable to store calibrated humidity data */
    uint16_t par_h1;
    /*! Variable to store calibrated humidity data */
    uint16_t par_h2;
    /*! Variable to store calibrated humidity data */
    int8_t par_h3;
    /*! Variable to store calibrated humidity data */
    int8_t par_h4;
    /*! Variable to store calibrated humidity data */
    int8_t par_h5;
    /*! Variable to store calibrated humidity data */
    uint8_t par_h6;
    /*! Variable to store calibrated humidity data */
    int8_t par_h7;
    /*! Variable to store calibrated gas data */
    int8_t par_gh1;
    /*! Variable to store calibrated gas data */
    int16_t par_gh2;
    /*! Variable to store calibrated gas data */
    int8_t par_gh3;
    /*! Variable to store calibrated temperature data */
    uint16_t par_t1;
    /*! Variable to store calibrated temperature data */
    int16_t par_t2;
    /*! Variable to store calibrated temperature data */
    int8_t par_t3;
    /*! Variable to store calibrated pressure data */
    uint16_t par_p1;
    /*! Variable to store calibrated pressure data */
    int16_t par_p2;
    /*! Variable to store calibrated pressure data */
    int8_t par_p3;
    /*! Variable to store calibrated pressure data */
    int16_t par_p4;
    /*! Variable to store calibrated pressure data */
    int16_t par_p5;
    /*! Variable to store calibrated pressure data */
    int8_t par_p6;
    /*! Variable to store calibrated pressure data */
    int8_t par_p7;
    /*! Variable to store calibrated pressure data */
    int16_t par_p8;
    /*! Variable to store calibrated pressure data */
    int16_t par_p9;
    /*! Variable to store calibrated pressure data */
    uint8_t par_p10;
 
#ifndef BME680_FLOAT_POINT_COMPENSATION
    /*! Variable to store t_fine size */
    int32_t t_fine;
#else
    /*! Variable to store t_fine size */
    float t_fine;
#endif
    /*! Variable to store heater resistance range */
    uint8_t res_heat_range;
    /*! Variable to store heater resistance value */
    int8_t res_heat_val;
    /*! Variable to store error range */
    int8_t range_sw_err;
};
 
/*!
 * @brief BME680 sensor settings structure which comprises of ODR,
 * over-sampling and filter settings.
 */
struct  bme680_tph_sett {
    /*! Humidity oversampling */
    uint8_t os_hum;
    /*! Temperature oversampling */
    uint8_t os_temp;
    /*! Pressure oversampling */
    uint8_t os_pres;
    /*! Filter coefficient */
    uint8_t filter;
};
 
/*!
 * @brief BME680 gas sensor which comprises of gas settings
 *  and status parameters
 */
struct  bme680_gas_sett {
    /*! Variable to store nb conversion */
    uint8_t nb_conv;
    /*! Variable to store heater control */
    uint8_t heatr_ctrl;
    /*! Run gas enable value */
    uint8_t run_gas;
    /*! Heater temperature value */
    uint16_t heatr_temp;
    /*! Duration profile value */
    uint16_t heatr_dur;
};
 
/*!
 * @brief BME680 device structure
 */
struct  bme680_dev {
    /*! Chip Id */
    uint8_t chip_id;
    /*! Device Id */
    uint8_t dev_id;
    /*! SPI/I2C interface */
    enum bme680_intf intf;
    /*! Memory page used */
    uint8_t mem_page;
    /*! Ambient temperature in Degree C */
    int8_t amb_temp;
    /*! Sensor calibration data */
    struct bme680_calib_data calib;
    /*! Sensor settings */
    struct bme680_tph_sett tph_sett;
    /*! Gas Sensor settings */
    struct bme680_gas_sett gas_sett;
    /*! Sensor power modes */
    uint8_t power_mode;
    /*! New sensor fields */
    uint8_t new_fields;
    /*! Store the info messages */
    uint8_t info_msg;
    /*! Bus read function pointer */
    bme680_com_fptr_t read;
    /*! Bus write function pointer */
    bme680_com_fptr_t write;
    /*! delay function pointer */
    bme680_delay_fptr_t delay_ms;
    /*! Communication function result */
    int8_t com_rslt;
};
#endif /* BME680_DEFS_H_ */


