/// \file mpu9250/driver.hpp
/// \brief Defines the mpu9250::driver class.
#ifndef MPU9250___DRIVER_H
#define MPU9250___DRIVER_H

#include <functional>

/// \brief Contains all code related to the MPU9250.
namespace mpu9250 {

/// \brief The base driver class for the MPU9250.
class driver
{
public:
    // ENUMERATIONS
    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers.
    enum class accel_dlpf_frequency
    {
        F_218HZ = 0x01,     ///< 218Hz cutoff frequency.
        F_99HZ = 0x02,      ///< 99Hz cutoff frequency.
        F_44HZ = 0x03,      ///< 44Hz cutoff frequency.
        F_21HZ = 0x04,      ///< 21Hz cutoff frequency.
        F_10HZ = 0x05,      ///< 10Hz cutoff frequency.
        F_5HZ = 0x06        ///< 218Hz cutoff frequency.
    };
    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the gyros.
    enum class gyro_dlpf_frequency
    {
        F_184HZ = 0x01,     ///< 184Hz cutoff frequency.
        F_92HZ = 0x02,      ///< 92Hz cutoff frequency.
        F_41HZ = 0x03,      ///< 41Hz cutoff frequency.
        F_20Hz = 0x04,      ///< 20Hz cutoff frequency.
        F_10Hz = 0x05,      ///< 10Hz cutoff frequency.
        F_5HZ = 0x06        ///< 5Hz cutoff frequency.
    };
    /// \brief Enumerates the full scale ranges (FSR) available for the accelerometers in g.
    enum class accel_fsr
    {
        G_2 = 0x00,         ///< +/- 2g range.
        G_4 = 0x01,         ///< +/- 4g range.
        G_8 = 0x02,         ///< +/- 8g range.
        G_16 = 0x03         ///< +/- 16g range.
    };
    /// \brief Enumerates the full scale ranges (FSR) available for the gyros in degrees/second.
    enum class gyro_fsr
    {
        DPS_250 = 0x00,     ///< +/- 250 deg/sec range.
        DPS_500 = 0x01,     ///< +/- 500 deg/sec range.
        DPS_1000 = 0x02,    ///< +/- 1000 deg/sec range.
        DPS_2000 = 0x03     ///< +/- 2000 deg/sec range.
    };

    // CLASSES
    /// \brief A structure for storing IMU data.
    struct data
    {
        /// \brief The X acceleration component (g).
        float accel_x;
        /// \brief The Y acceleration component (g).
        float accel_y;
        /// \brief The Z acceleration component (g).
        float accel_z;
        /// \brief The X angular velocity component (deg/s).
        float gyro_x;
        /// \brief The Y angular velocity component (deg/s).
        float gyro_y;
        /// \brief The Z angular velocity component (deg/s).
        float gyro_z;
        /// \brief The X magnetic field component (uT).
        float magneto_x;
        /// \brief The Y magnetic field component (uT).
        float magneto_y;
        /// \brief The Z magnetic field component (uT).
        float magneto_z;
        /// \brief The die temperature of the IMU (deg C).
        float temp;
    };

    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    driver();

    // CALLBACK
    /// \brief Attaches a callback to handle data when it becomes available.
    /// \param callback The callback function to execute.
    void set_data_callback(std::function<void(driver::data)> callback);

    // INITIALIZATION
    /// \brief Initializes the MPU9250.
    /// \param i2c_bus The I2C bus to communicate with the MPU9250 over.
    /// \param i2c_address The I2C address of the MPU9250.
    /// \param interrupt_gpio_pin The GPIO pin connected to the MPU9250's interrupt pin.
    void initialize(uint32_t i2c_bus, uint32_t i2c_address, uint32_t interrupt_gpio_pin);
    /// \brief Deinitializes the MPU9250.
    void deinitialize();

    // CONFIGURATION
    /// \brief Configures the MPU9250's gyroscope.
    /// \param fsr The desired full-scale range (FSR).
    /// \param dlpf_frequency The desired cut-off frequency of the digital low-pass filter (DLPF).
    void configure_gyro(gyro_fsr fsr, gyro_dlpf_frequency dlpf_frequency);
    /// \brief Configures the MPU9250's accelerometer.
    /// \param fsr The desired full-scale range (FSR).
    /// \param dlpf_frequency The desired cut-off frequency of the digital low-pass filter (DLPF).
    void configure_accel(accel_fsr fsr, accel_dlpf_frequency dlpf_frequency);
    /// \brief Configures the sample rate for the MPU9250.
    /// \param divider The divider to apply to the base clock. Sample rate = 1000Hz / (1 + divider).
    void configure_sample_rate(uint8_t divider);

protected:
    // ENUMERATIONS
    /// \brief Enumerates the MPU9250 register addresses.
    enum class register_mpu9250
    {
        SAMPLE_RATE_DIVIDER = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        ACCEL_CONFIG_2 = 0x1D,
        I2C_MASTER_CONTROL = 0x36,
        INT_BYP_CFG = 0x37,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3A,
        ACCEL_X_HIGH = 0x3B,
        ACCEL_X_LOW = 0x3C,
        ACCEL_Y_HIGH = 0x3D,
        ACCEL_Y_LOW = 0x3E,
        ACCEL_Z_HIGH = 0x3F,
        ACCEL_Z_LOW = 0x40,
        TEMP_HIGH = 0x41,
        TEMP_LOW = 0x42,
        GYRO_X_HIGH = 0x43,
        GYRO_X_LOW = 0x44,
        GYRO_Y_HIGH = 0x45,
        GYRO_Y_LOW = 0x46,
        GYRO_Z_HIGH = 0x47,
        GYRO_Z_LOW = 0x48,
        PWR_MGMT_1 = 0x6B,
        WHO_AM_I = 0x75
    };
    /// \brief Enumerates the AK8963 register addresses.
    enum class register_ak8963
    {
        WHO_AM_I = 0x00,
        X_LOW = 0x03,
        X_HIGH = 0x04,
        Y_LOW = 0x05,
        Y_HIGH = 0x06,
        Z_LOW = 0x07,
        Z_HIGH = 0x08,
        STATUS_2 = 0x09,
        CONTROL_1 = 0x0A
    };

    // I2C
    /// \brief Initializes the I2C and GPIO interface of the driver.
    /// \param i2c_bus The I2C bus to interface with the MPU9250 over.
    /// \param i2c_address The I2C address of the MPU9250.
    /// \param interrupt_gpio_pin The GPIO input pin that is connected to the MPU9250 interrupt pin.
    virtual void initialize_i2c(uint32_t i2c_bus, uint32_t i2c_address, uint32_t interrupt_gpio_pin) = 0;
    /// \brief Deinitialies the I2C interface of the driver.
    virtual void deinitialize_i2c() = 0;
    /// \brief Writes data to a register on the MPU9250.
    /// \param address The address of the register to write to.
    /// \param value The data to write to the register.
    virtual void write_mpu9250_register(register_mpu9250 address, uint8_t value) = 0;
    /// \brief Reads data from a register on the MPU9250.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    virtual uint8_t read_mpu9250_register(register_mpu9250 address) = 0;
    /// \brief Block reads data from several registers on the MPU9250.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    virtual void read_mpu9250_registers(register_mpu9250 address, uint32_t n_bytes, uint8_t* buffer) = 0;
    /// \brief Writes data to a register on the AK8963.
    /// \param address The address of the register to write to.
    /// \param value The data to write to the register.
    virtual void write_ak8963_register(register_ak8963 address, uint8_t value) = 0;
    /// \brief Reads data from a register on the AK8963.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    virtual uint8_t read_ak8963_register(register_ak8963 address) = 0;
    /// \brief Block reads data from several registers on the AK8963.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    virtual void read_ak8963_registers(register_ak8963 address, uint32_t n_bytes, uint8_t* buffer) = 0;

    // ACCESS
    /// \brief Reads all IMU data directly from the MPU9250 and AK8963 and raises the data callback.
    void read_data();

private:
    // FSR
    /// \brief Stores the current FSR of the accelerometer.
    float m_fsr_accel;
    /// \brief Stores the current FSR of the gyroscope.
    float m_fsr_gyro;

    // CALLBACKS
    /// \brief m_data_callback The callback function to execute when IMU data is read.
    std::function<void(driver::data)> m_data_callback;

    // SERIALIZATION
    /// \brief Indicates if the system is little endian.
    bool m_little_endian;
    /// \brief Deserializes an int16 from a big-endian byte array.
    /// \param bytes The big-endian byte array to read from.
    /// \returns The deserialized int16.
    int16_t deserialize_be(uint8_t* bytes) const;
    /// \brief Deserializes an int16 from a little-endian byte array.
    /// \param bytes The little-endian byte array to read from.
    /// \returns The deserialized int16.
    int16_t deserialize_le(uint8_t* bytes) const;
};

}

#endif