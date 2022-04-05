/// \file mpu9250/configuration/accel.hpp
/// \brief Defines various accelerometer configuration enumerations.
#ifndef MPU9250___CONFIGURATION___ACCEL_H
#define MPU9250___CONFIGURATION___ACCEL_H

namespace mpu9250 {

namespace configuration {

/// \brief Contains all code related to MPU9250 accelerometer configuration.
namespace accel {

/// \brief Enumerates the full scale ranges (FSR) available for the accelerometers in g.
enum class fsr
{
    G_2 = 0x00,         ///< +/- 2g range.
    G_4 = 0x01,         ///< +/- 4g range.
    G_8 = 0x02,         ///< +/- 8g range.
    G_16 = 0x03         ///< +/- 16g range.
};

/// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers.
enum class dlpf_frequency
{
    F_218HZ = 0x01,     ///< 218Hz cutoff frequency.
    F_99HZ = 0x02,      ///< 99Hz cutoff frequency.
    F_44HZ = 0x03,      ///< 44Hz cutoff frequency.
    F_21HZ = 0x04,      ///< 21Hz cutoff frequency.
    F_10HZ = 0x05,      ///< 10Hz cutoff frequency.
    F_5HZ = 0x06        ///< 218Hz cutoff frequency.
};    

}}}

#endif