/// \file mpu9250/measurement.hpp
/// \brief Defines the mpu9250::measurement struct.
#ifndef MPU9250___MEASUREMENT_H
#define MPU9250___MEASUREMENT_H

namespace mpu9250 {

/// \brief A measurement from the MPU9250.
struct measurement
{
    // OBJECTS
    /// \brief A 3D vector.
    struct vector
    {
        /// \brief The x component of the vector.
        float x;
        /// \brief The y component of the vector.
        float y;
        /// \brief The z component of the vector.
        float z;
    };

    // FIELDS
    /// \brief The measured acceleration (g).
    vector accel;
    /// \brief The measured angular velocity (deg/s).
    vector gyro;
    /// \brief The measured magnetic field (uT).
    vector magneto;
    /// \brief The die temperature of the IMU (deg C).
    float temp;
};

}

#endif