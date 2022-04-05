/// \file mpu9250/registers/mpu9250.hpp
/// \brief Defines the mpu9250::registers::mpu9250 enumeration.
#ifndef MPU9250___REGISTERS___MPU9250_H
#define MPU9250___REGISTERS___MPU9250_H

namespace mpu9250 {

/// \brief Contains all code related to enumeration of MPU9250 registers.
namespace registers {

/// \brief Enumerates the MPU9250 register addresses.
enum class mpu9250
{
    SAMPLE_RATE_DIVIDER = 0x19,     ///< The sample rate divider applied to the sample frequency clock.
    CONFIG = 0x1A,                  ///< FIFO, FSYNC, and gyro DLPF settings.
    GYRO_CONFIG = 0x1B,             ///< Gyro FSR and DLPF mode settings.
    ACCEL_CONFIG = 0x1C,            ///< Accel FSR settings.
    ACCEL_CONFIG_2 = 0x1D,          ///< Accel DLPF settings.
    INT_BYP_CFG = 0x37,             ///< Interrupt pin and I2C bypass settings.
    INT_ENABLE = 0x38,              ///< Interrupt enable settings.
    ACCEL_X_HIGH = 0x3B,            ///< Accel x data high.
    ACCEL_X_LOW = 0x3C,             ///< Accel x data low.
    ACCEL_Y_HIGH = 0x3D,            ///< Accel y data high.
    ACCEL_Y_LOW = 0x3E,             ///< Accel y data low.
    ACCEL_Z_HIGH = 0x3F,            ///< Accel z data high.
    ACCEL_Z_LOW = 0x40,             ///< Accel z data low.
    TEMP_HIGH = 0x41,               ///< Temp data high.
    TEMP_LOW = 0x42,                ///< Temp data low.
    GYRO_X_HIGH = 0x43,             ///< Gyro x data high.
    GYRO_X_LOW = 0x44,              ///< Gyro x data low.
    GYRO_Y_HIGH = 0x45,             ///< Gyro y data high.
    GYRO_Y_LOW = 0x46,              ///< Gyro y data low.
    GYRO_Z_HIGH = 0x47,             ///< Gyro z data high.
    GYRO_Z_LOW = 0x48,              ///< Gyro z data low.
    PWR_MGMT_1 = 0x6B,              ///< Power settings.
    WHO_AM_I = 0x75                 ///< Identification.
};

}}

#endif