/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/bus.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_bmi160.h"

#if defined(USE_GYRO_BMI160) || defined(USE_ACC_BMI160)

/* BMI160 Registers */
#define BMI160_REG_CHIPID           0x00
#define BMI160_REG_PMU_STAT         0x03
#define BMI160_REG_GYR_DATA_X_LSB   0x0C
#define BMI160_REG_ACC_DATA_X_LSB   0x12
#define BMI160_REG_STATUS           0x1B
#define BMI160_REG_TEMPERATURE_0    0x20
#define BMI160_REG_ACC_CONF         0x40
#define BMI160_REG_ACC_RANGE        0x41
#define BMI160_REG_GYR_CONF         0x42
#define BMI160_REG_GYR_RANGE        0x43
#define BMI160_REG_INT_EN1          0x51
#define BMI160_REG_INT_OUT_CTRL     0x53
#define BMI160_REG_INT_MAP1         0x56
#define BMI160_REG_FOC_CONF         0x69
#define BMI160_REG_CONF             0x6A
#define BMI160_REG_OFFSET_0         0x77
#define BMI160_REG_CMD              0x7E

/* Register values */
#define BMI160_PMU_CMD_PMU_ACC_NORMAL   0x11
#define BMI160_PMU_CMD_PMU_GYR_NORMAL   0x15
#define BMI160_INT_EN1_DRDY             0x10
#define BMI160_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMI160_REG_INT_MAP1_INT1_DRDY   0x80
#define BMI160_CMD_START_FOC            0x03
#define BMI160_CMD_PROG_NVM             0xA0
#define BMI160_REG_STATUS_NVM_RDY       0x10
#define BMI160_REG_STATUS_FOC_RDY       0x08
#define BMI160_REG_CONF_NVM_PROG_EN     0x02

#define BMI160_BWP_NORMAL               0x20
#define BMI160_BWP_OSR2                 0x10
#define BMI160_BWP_OSR4                 0x00

#define BMI160_ODR_400_Hz               0x0A
#define BMI160_ODR_800_Hz               0x0B
#define BMI160_ODR_1600_Hz              0x0C
#define BMI160_ODR_3200_Hz              0x0D

#define BMI160_RANGE_2G                 0x03
#define BMI160_RANGE_4G                 0x05
#define BMI160_RANGE_8G                 0x08
#define BMI160_RANGE_16G                0x0C

#define BMI160_RANGE_125DPS             0x04
#define BMI160_RANGE_250DPS             0x03
#define BMI160_RANGE_500DPS             0x02
#define BMI160_RANGE_1000DPS            0x01
#define BMI160_RANGE_2000DPS            0x00

typedef struct __attribute__ ((__packed__)) bmi160ContextData_s {
    uint16_t    chipMagicNumber;
    uint8_t     lastReadStatus;
    uint8_t     __padding;
    uint8_t     gyroRaw[6];
    uint8_t     accRaw[6];
} bmi160ContextData_t;

static void bmi160AccAndGyroInit(gyroDev_t *gyro)
{
    uint8_t value;
    mpuIntExtiInit(gyro);

    busSetSpeed(gyro->busDev, BUS_SPEED_INITIALIZATION);

    // Normal power mode, can take up to 80+3.8ms
    busWrite(gyro->busDev, BMI160_REG_CMD, BMI160_PMU_CMD_PMU_GYR_NORMAL);
    delay(100);

    busWrite(gyro->busDev, BMI160_REG_CMD, BMI160_PMU_CMD_PMU_ACC_NORMAL);
    delay(5);

    // Verify that normal power mode was entered
    busRead(gyro->busDev, BMI160_REG_PMU_STAT, &value);
    if ((value & 0x3C) != 0x14) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }

    // Set ranges and ODR
    busWrite(gyro->busDev, BMI160_REG_ACC_CONF, BMI160_BWP_OSR4 | BMI160_ODR_400_Hz);
    delay(1);

    // Figure out suitable filter configuration
    if (gyro->lpf == GYRO_LPF_256HZ) {
        switch (gyro->sampleRateDenom) {
            case 1: // Invensense rate 8kHz   - Bosch N/A
            case 2: // Invensense rate 4kHz   - Bosch 3.2kHz
            case 3: // Invensense rate 2.7kHz - Bosch 3.2kHz
                value = BMI160_BWP_NORMAL | BMI160_ODR_3200_Hz;     // ODR = 3.2 kHz, LPF = 890 Hz
                gyro->sampleRateIntervalUs = 1000000 / 3200;
                break;

            case 4: // Invensense rate 2kHz   - Bosch 1.6kHz
            case 5: // Invensense rate 1.6kHz - Bosch 1.6kHz
            case 6: // Invensense rate 1.3kHz - Bosch 1.6kHz
                value = BMI160_BWP_NORMAL | BMI160_ODR_1600_Hz;     // ODR = 1.6 kHz, LPF = 524 Hz
                gyro->sampleRateIntervalUs = 1000000 / 1600;
                break;

            case 7: // Invensense rate 1.1kHz - Bosch 800Hz
            case 8: // Invensense rate 1kHz   - Bosch 800Hz
            default:    // Fallback value
                value = BMI160_BWP_NORMAL | BMI160_ODR_800_Hz;      // ODR = 0.8 kHz, LPF = 255 Hz
                gyro->sampleRateIntervalUs = 1000000 / 800;
                break;
        }
    }
    else if (gyro->lpf == GYRO_LPF_188HZ) {
        if (gyro->sampleRateDenom == 1) {
            value = BMI160_BWP_OSR2 | BMI160_ODR_800_Hz;            // ODR = 800 Hz, LPF = 128 Hz
            gyro->sampleRateIntervalUs = 1000000 / 800;
        }
        else {
            value = BMI160_BWP_NORMAL | BMI160_ODR_400_Hz;          // ODR = 400 Hz, LPF = 137 Hz
            gyro->sampleRateIntervalUs = 1000000 / 400;
        }
    }
    else if (gyro->lpf == GYRO_LPF_98HZ) {
        if (gyro->sampleRateDenom == 1) {
            value = BMI160_BWP_OSR4 | BMI160_ODR_800_Hz;            // ODR = 800 Hz, LPF = 63 Hz
            gyro->sampleRateIntervalUs = 1000000 / 800;
        }
        else {
            value = BMI160_BWP_OSR2 | BMI160_ODR_400_Hz;            // ODR = 400 Hz, LPF = 68 Hz
            gyro->sampleRateIntervalUs = 1000000 / 400;
        }
    }
    else {
        if (gyro->sampleRateDenom == 1) {
            value = BMI160_BWP_OSR4 | BMI160_ODR_800_Hz;            // ODR = 800 Hz, LPF = 63 Hz
            gyro->sampleRateIntervalUs = 1000000 / 800;
        }
        else {
            value = BMI160_BWP_OSR4 | BMI160_ODR_400_Hz;            // ODR = 400 Hz, LPF = 34 Hz
            gyro->sampleRateIntervalUs = 1000000 / 400;
        }
    }

    busWrite(gyro->busDev, BMI160_REG_GYR_CONF, value);
    delay(1);

    busWrite(gyro->busDev, BMI160_REG_ACC_RANGE, BMI160_RANGE_8G);
    delay(1);
        
    busWrite(gyro->busDev, BMI160_REG_GYR_RANGE, BMI160_RANGE_2000DPS);
    delay(1);    

    // Enable offset compensation
    // uint8_t val = spiBusReadRegister(bus, BMI160_REG_OFFSET_0);
    // busWrite(gyro->busDev, BMI160_REG_OFFSET_0, val | 0xC0);    

    // Enable data ready interrupt
    busWrite(gyro->busDev, BMI160_REG_INT_EN1, BMI160_INT_EN1_DRDY);
    delay(1);

    // Enable INT1 pin
    busWrite(gyro->busDev, BMI160_REG_INT_OUT_CTRL, BMI160_INT_OUT_CTRL_INT1_CONFIG);
    delay(1);

    // Map data ready interrupt to INT1 pin
    busWrite(gyro->busDev, BMI160_REG_INT_MAP1, BMI160_REG_INT_MAP1_INT1_DRDY);
    delay(1);

    busSetSpeed(gyro->busDev, BUS_SPEED_FAST);
}

bool bmi160GyroReadScratchpad(gyroDev_t *gyro)
{
    bmi160ContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->lastReadStatus = busReadBuf(gyro->busDev, BMI160_REG_GYR_DATA_X_LSB, ctx->gyroRaw, 6 + 6);

    if (ctx->lastReadStatus) {
        gyro->gyroADCRaw[X] = (int16_t)((ctx->gyroRaw[1] << 8) | ctx->gyroRaw[0]);
        gyro->gyroADCRaw[Y] = (int16_t)((ctx->gyroRaw[3] << 8) | ctx->gyroRaw[2]);
        gyro->gyroADCRaw[Z] = (int16_t)((ctx->gyroRaw[5] << 8) | ctx->gyroRaw[4]);

        return true;
    }

    return false;
}

bool bmi160AccReadScratchpad(accDev_t *acc)
{
    bmi160ContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);

    if (ctx->lastReadStatus) {
        acc->ADCRaw[X] = (int16_t)((ctx->accRaw[1] << 8) | ctx->accRaw[0]);
        acc->ADCRaw[Y] = (int16_t)((ctx->accRaw[3] << 8) | ctx->accRaw[2]);
        acc->ADCRaw[Z] = (int16_t)((ctx->accRaw[5] << 8) | ctx->accRaw[4]);
        return true;
    }

    return false;
}

static void bmi160AccInit(accDev_t *acc)
{
    acc->acc_1G = 4096;
}

bool bmi160AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_BMI160, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    bmi160ContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0xB160) {
        return false;
    }

    acc->initFn = bmi160AccInit;
    acc->readFn = bmi160AccReadScratchpad;

    return true;
}

static bool deviceDetect(busDevice_t * busDev)
{
    uint8_t attempts;

    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    for (attempts = 0; attempts < 5; attempts++) {
        uint8_t chipId;

        delay(100);
        busRead(busDev, BMI160_REG_CHIPID, &chipId);

        if (chipId == 0xD1) {
            return true;
        }
    }

    return false;
}

bool bmi160GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_BMI160, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected BMI160 gyro
    bmi160ContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0xB160;

    gyro->initFn = bmi160AccAndGyroInit;
    gyro->readFn = bmi160GyroReadScratchpad;
    gyro->intStatusFn = mpuCheckDataReady;
    gyro->temperatureFn = NULL;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif