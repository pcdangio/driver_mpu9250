/// \file mpu9250/driver.hpp
/// \brief Defines the mpu9250::driver class.
#ifndef MPU9250___DRIVER_H
#define MPU9250___DRIVER_H

#include <mpu9250/configuration/gyro.hpp>
#include <mpu9250/configuration/accel.hpp>
#include <mpu9250/registers/mpu9250.hpp>
#include <mpu9250/registers/ak8936.hpp>
#include <mpu9250/measurement.hpp>

#include <functional>

/// \brief Contains all code related to the MPU9250.
namespace mpu9250 {

/// \brief The base driver class for the MPU9250.
class driver
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    driver();

    // CALLBACK
    /// \brief Attaches a callback to handle a new measurement when it becomes available.
    /// \param callback The callback function to execute.
    void set_measurement_callback(std::function<void(measurement)> callback);

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
    void configure_gyro(configuration::gyro::fsr fsr, configuration::gyro::dlpf_frequency dlpf_frequency);
    /// \brief Configures the MPU9250's accelerometer.
    /// \param fsr The desired full-scale range (FSR).
    /// \param dlpf_frequency The desired cut-off frequency of the digital low-pass filter (DLPF).
    void configure_accel(configuration::accel::fsr fsr, configuration::accel::dlpf_frequency dlpf_frequency);
    /// \brief Configures the sample rate for the MPU9250.
    /// \param divider The divider to apply to the base clock. Sample rate = 1000Hz / (1 + divider).
    void configure_sample_rate(uint8_t divider);

protected:
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
    virtual void write_mpu9250_register(registers::mpu9250 address, uint8_t value) = 0;
    /// \brief Reads data from a register on the MPU9250.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    virtual uint8_t read_mpu9250_register(registers::mpu9250 address) = 0;
    /// \brief Block reads data from several registers on the MPU9250.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    virtual void read_mpu9250_registers(registers::mpu9250 address, uint32_t n_bytes, uint8_t* buffer) = 0;
    /// \brief Writes data to a register on the AK8963.
    /// \param address The address of the register to write to.
    /// \param value The data to write to the register.
    virtual void write_ak8963_register(registers::ak8963 address, uint8_t value) = 0;
    /// \brief Reads data from a register on the AK8963.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    virtual uint8_t read_ak8963_register(registers::ak8963 address) = 0;
    /// \brief Block reads data from several registers on the AK8963.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    virtual void read_ak8963_registers(registers::ak8963 address, uint32_t n_bytes, uint8_t* buffer) = 0;

    // ACCESS
    /// \brief Reads the last measurement directly from the MPU9250 and AK8963 and raises the measurement callback.
    void read_measurement();

private:
    // FSR
    /// \brief Stores the current FSR of the accelerometer.
    float m_fsr_accel;
    /// \brief Stores the current FSR of the gyroscope.
    float m_fsr_gyro;

    // CALLBACKS
    /// \brief The callback function to execute when a new measurement is read.
    std::function<void(measurement)> m_measurement_callback;

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