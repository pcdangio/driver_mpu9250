/// \file mpu9250/configuration/gyro.hpp
/// \brief Defines various gyroscope configuration enumerations.
#ifndef MPU9250___CONFIGURATION___GYRO_H
#define MPU9250___CONFIGURATION___GYRO_H

namespace mpu9250 {

/// \brief Contains all code related to MPU9250 configuration.
namespace configuration {

/// \brief Contains all code related to MPU9250 gyroscope configuration.
namespace gyro {

/// \brief Enumerates the full scale ranges (FSR) available for the gyros in degrees/second.
enum class fsr
{
    DPS_250 = 0x00,     ///< +/- 250 deg/sec range.
    DPS_500 = 0x01,     ///< +/- 500 deg/sec range.
    DPS_1000 = 0x02,    ///< +/- 1000 deg/sec range.
    DPS_2000 = 0x03     ///< +/- 2000 deg/sec range.
};

/// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the gyros.
enum class dlpf_frequency
{
    F_184HZ = 0x01,     ///< 184Hz cutoff frequency.
    F_92HZ = 0x02,      ///< 92Hz cutoff frequency.
    F_41HZ = 0x03,      ///< 41Hz cutoff frequency.
    F_20Hz = 0x04,      ///< 20Hz cutoff frequency.
    F_10Hz = 0x05,      ///< 10Hz cutoff frequency.
    F_5HZ = 0x06        ///< 5Hz cutoff frequency.
};

}}}

#endif