/// \file mpu9250/registers/ak8963.hpp
/// \brief Defines the mpu9250::registers::ak8963 enumeration.
#ifndef MPU9250___REGISTERS___AK8963_H
#define MPU9250___REGISTERS___AK8963_H

namespace mpu9250 {

namespace registers {

/// \brief Enumerates the AK8963 register addresses.
enum class ak8963
{
    WHO_AM_I = 0x00,        ///< Identification.
    X_LOW = 0x03,           ///< X data low.
    X_HIGH = 0x04,          ///< X data high.
    Y_LOW = 0x05,           ///< Y data low.
    Y_HIGH = 0x06,          ///< Y data high.
    Z_LOW = 0x07,           ///< Z data low.
    Z_HIGH = 0x08,          ///< Z data high.
    STATUS_2 = 0x09,        ///< Status of last measurement.
    CONTROL_1 = 0x0A        ///< Power and operating mode.
};

}}

#endif