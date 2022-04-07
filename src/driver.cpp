#include <mpu9250/driver.hpp>

#include <stdexcept>
#include <unistd.h>
#include <cmath>
#include <limits>

using namespace mpu9250;

// CONSTRUCTORS
driver::driver()
{
    // Check machine endianness.
    uint32_t integer = 1;
    driver::m_little_endian = reinterpret_cast<uint8_t*>(&integer)[0];
}

// CONFIGURATION
void driver::set_measurement_callback(std::function<void(measurement)> callback)
{
    // Lock callback thread protection.
    driver::m_mutex_callback.lock();

    // Store the callback.
    driver::m_measurement_callback = callback;

    // Unlock callback thread protection.
    driver::m_mutex_callback.unlock();
}

// INITIALIZATION
void driver::initialize(uint32_t i2c_bus, uint32_t i2c_address)
{
    // Initialize I2C.
    initialize_i2c(i2c_bus, i2c_address);

    // Test MPU9250 communications.
    if(read_mpu9250_register(registers::mpu9250::WHO_AM_I) != 0x71)
    {
        throw std::runtime_error("connected device is not an mpu9250");
    }

    // Configure MPU9250.
    // Power on MPU9250 sensors and reset default settings.
    write_mpu9250_register(registers::mpu9250::PWR_MGMT_1, 0x80);
    // Sleep for 100ms to let gyro PLL to stabilize before selecting it as clock source.
    usleep(100000);
    // Change clock source to PLL from gyro.
    write_mpu9250_register(registers::mpu9250::PWR_MGMT_1, 0x01);
    // Enable I2C bypass mode for access to AK8963.
    write_mpu9250_register(registers::mpu9250::INT_BYP_CFG, 0x02);
    // Enable interrupt pin for raw data ready.
    write_mpu9250_register(registers::mpu9250::INT_ENABLE, 0x01);

    // Test AK8963 communications.
    if(read_ak8963_register(registers::ak8963::WHO_AM_I) != 0x48)
    {
        throw std::runtime_error("connected device does not have an ak8963");
    }

    // Configure AK8963.
    // Power on magnetometer at 16bit resolution with 100Hz sample rate.
    // NOTE: continuous mode 1 = 8Hz, 2 = 100Hz.
    write_ak8963_register(registers::ak8963::CONTROL_1, 0x16);

    // Store default FSR values for data conversion.
    driver::m_fsr_gyro = 250.0;
    driver::m_fsr_accel = 2.0;
}
void driver::deinitialize()
{
    // Power down the AK8963.
    write_ak8963_register(registers::ak8963::CONTROL_1, 0x00);

    // Power down the MPU9250.
    write_mpu9250_register(registers::mpu9250::PWR_MGMT_1, 0x40);

    // Deinit I2C.
    deinitialize_i2c();
}

// CONFIGURATION
void driver::configure_gyro(configuration::gyro::fsr fsr, configuration::gyro::dlpf_frequency dlpf_frequency)
{
    // Set FSR, and set F_CHOICE_B to 0b00 to enable DLPF.
    write_mpu9250_register(registers::mpu9250::GYRO_CONFIG, static_cast<uint8_t>(fsr) << 3);

    // Set DLPF frequency.
    // NOTE: This also sets FIFO_MODE and EXT_SYNC_SET to zero.
    write_mpu9250_register(registers::mpu9250::CONFIG, static_cast<uint8_t>(dlpf_frequency));

    // Store FSR for data conversion.
    switch(fsr)
    {
        case configuration::gyro::fsr::DPS_250:
        {
            driver::m_fsr_gyro = 250.0;
            break;
        }
        case configuration::gyro::fsr::DPS_500:
        {
            driver::m_fsr_gyro = 500.0;
            break;
        }
        case configuration::gyro::fsr::DPS_1000:
        {
            driver::m_fsr_gyro = 1000.0;
            break;
        }
        case configuration::gyro::fsr::DPS_2000:
        {
            driver::m_fsr_gyro = 2000.0;
            break;
        }
    }
}
void driver::configure_accel(configuration::accel::fsr fsr, configuration::accel::dlpf_frequency dlpf_frequency)
{
    // Set FSR.
    write_mpu9250_register(registers::mpu9250::ACCEL_CONFIG, static_cast<uint8_t>(fsr) << 3);

    // Set DLPF and set F_CHOICE_B to 0b0 to enable DLPF.
    write_mpu9250_register(registers::mpu9250::ACCEL_CONFIG_2, static_cast<uint8_t>(dlpf_frequency));

    // Store FSR for data conversion.
    switch(fsr)
    {
        case configuration::accel::fsr::G_2:
        {
            driver::m_fsr_accel = 2.0;
            break;
        }
        case configuration::accel::fsr::G_4:
        {
            driver::m_fsr_accel = 4.0;
            break;
        }
        case configuration::accel::fsr::G_8:
        {
            driver::m_fsr_accel = 8.0;
            break;
        }
        case configuration::accel::fsr::G_16:
        {
            driver::m_fsr_accel = 16.0;
            break;
        }
    }
}
void driver::configure_sample_rate(uint8_t divider)
{
    // Set sample rate.
    write_mpu9250_register(registers::mpu9250::SAMPLE_RATE_DIVIDER, divider);
}

// METHODS
void driver::read_measurement()
{
    // Burst read from the MPU9250.
    // Create buffer for accel/temp/gyro data.
    uint8_t atg_buffer[14];
    // Create buffer for magneto data.
    uint8_t magneto_buffer[7];
    try
    {
        read_mpu9250_registers(registers::mpu9250::ACCEL_X_HIGH, 14, atg_buffer);
        read_ak8963_registers(registers::ak8963::X_LOW, 7, magneto_buffer);
    }
    catch(const std::exception& e)
    {
        // Quit before callback. Do not report error in loop.
        return;
    }

    // Create measurement structure.
    measurement measurement;

    // Parse out accel data.
    measurement.accel.x = driver::m_fsr_accel * static_cast<float>(driver::deserialize_be(&atg_buffer[0])) / 32767.0f;
    measurement.accel.y = driver::m_fsr_accel * static_cast<float>(driver::deserialize_be(&atg_buffer[2])) / 32767.0f;
    measurement.accel.z = driver::m_fsr_accel * static_cast<float>(driver::deserialize_be(&atg_buffer[4])) / 32767.0f;

    // Parse out temperature data.
    // Formula is DegC = ((raw - roomtemp_offset)/temp_sensitivity) + 21
    // From datasheet, roomtemp_offset = 0, temp_sensitivity = 333.87
    measurement.temp = static_cast<float>(driver::deserialize_be(&atg_buffer[6])) / 333.87f + 21.0f;
    
    // Parse out gyro data.
    measurement.gyro.x = driver::m_fsr_gyro * static_cast<float>(driver::deserialize_be(&atg_buffer[8])) / 32767.0f;
    measurement.gyro.y = driver::m_fsr_gyro * static_cast<float>(driver::deserialize_be(&atg_buffer[10])) / 32767.0f;
    measurement.gyro.z = driver::m_fsr_gyro * static_cast<float>(driver::deserialize_be(&atg_buffer[12])) / 32767.0f;
    
    // Check if there was a magnetic overflow.
    if(magneto_buffer[6] & 0x08)
    {
        // Magnetic overflow occured and data is invalid.
        measurement.magneto.x = std::numeric_limits<float>::quiet_NaN();
        measurement.magneto.y = std::numeric_limits<float>::quiet_NaN();
        measurement.magneto.z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
        // Store measurements.
        // Magneto FSR is +/-4912 uT, with max range of +/-32760.
        measurement.magneto.x = 4912.0f * static_cast<float>(driver::deserialize_le(&magneto_buffer[0])) / 32760.0f;
        measurement.magneto.y = 4912.0f * static_cast<float>(driver::deserialize_le(&magneto_buffer[2])) / 32760.0f;
        measurement.magneto.z = 4912.0f * static_cast<float>(driver::deserialize_le(&magneto_buffer[4])) / 32760.0f;
    }

    // Initiate the measurement callback.

    // Lock callback thread protection.
    driver::m_mutex_callback.lock();

    // Call callback.
    driver::m_measurement_callback(measurement);

    // Unlock callback thread protection.
    driver::m_mutex_callback.unlock();
}

// SERIALIZATION
int16_t driver::deserialize_be(uint8_t* bytes) const
{
    // Create output.
    int16_t output;

    // Check endianness.
    if(driver::m_little_endian)
    {
        output = (bytes[0] << 8) | bytes[1];
    }
    else
    {
        output = (bytes[1] << 8) | bytes[0];
    }

    return output;
}
int16_t driver::deserialize_le(uint8_t* bytes) const
{
    // Create output.
    int16_t output;

    // Check endianness.
    if(driver::m_little_endian)
    {
        output = (bytes[1] << 8) | bytes[0];
    }
    else
    {
        output = (bytes[0] << 8) | bytes[1];
    }

    return output;
}