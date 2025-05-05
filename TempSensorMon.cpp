
#include <iostream>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <unistd.h>

#define EEPROM_ADDRESS 0x50

#define HW_REV_A 0x00
#define HW_REV_B 0x01

#define HW_REV HW_REV_A
#define SERIAL_NUM_ADDR 0x01
#define SERIAL_NUM_MAX_LENGTH 16
#define TEMP_WARNING_THRESHOLD 85.0f
#define TEMP_CRITICAL_HIGH_THRESHOLD 105.0f
#define TEMP_CRITICAL_LOW_THRESHOLD 5.0f

#define GREEN_LED_PIN 10
#define YELLOW_LED_PIN 11
#define RED_LED_PIN 12

#define TIMER_SIGNAL SIGALRM // Use SIGALRM for timer simulation

class SystemConfig {
public:
    uint8_t hardware_revision;
    char serial_number[SERIAL_NUM_MAX_LENGTH];

    SystemConfig() : hardware_revision(0) {
        std::memset(serial_number, 0, SERIAL_NUM_MAX_LENGTH);
    }
};

enum TempState {
    TEMP_OK,
    TEMP_WARNING,
    TEMP_CRITICAL
};

class TempSensorMonitoring {
private:
    SystemConfig current_config;
    volatile uint16_t latest_adc_value;
    volatile bool new_adc_value_ready;
    TempState current_temp_state;

public:
    TempSensorMonitoring() : latest_adc_value(0), new_adc_value_ready(false), current_temp_state(TEMP_OK) {}

    void gpioInit(uint32_t pin) {
        std::cout << "GPIO Pin " << pin << " Initialized as output" << std::endl;
    }

    void gpioSet(uint32_t pin, uint8_t state) {
        std::cout << "GPIO Pin " << pin << " set" << std::endl;
    }

    void i2cInit(uint32_t speed) {
        std::cout << "I2C Initialized at " << speed << " Hz" << std::endl;
    }

    uint8_t i2cReadByte(uint8_t device_address, uint8_t memory_address) {
        std::cout << "I2C Read Byte from 0x" << std::hex << (int)device_address
                  << ", Address 0x" << (int)memory_address << std::endl;

        return HW_REV;
    }

    void i2cReadBlock(uint8_t device_address, uint8_t memory_address, char *data, uint16_t length) {
        std::cout << "I2C Read Block from 0x" << std::hex << (int)device_address
                  << ", Address 0x" << (int)memory_address << ", Length " << length << std::endl;
        std::snprintf(data, length, "ABC1234");
    }

    void adcInit() {
        std::cout << "ADC Initialized" << std::endl;
    }

    void adcStartConversion() {
        // Simulate ADC conversion start
    }

    bool adcConversionComplete() {
        return true; 
    }

    uint16_t adcReadRaw() {
        static int16_t adc_value = -200; 
        adc_value += 80;
        if (adc_value > 1500) {
            adc_value = -200;
        }
        return static_cast<uint16_t>(adc_value);
    }

    void configInit() {
        // Read hardware revision
        current_config.hardware_revision = i2cReadByte(EEPROM_ADDRESS, HW_REV);

        i2cReadBlock(EEPROM_ADDRESS, SERIAL_NUM_ADDR, current_config.serial_number, SERIAL_NUM_MAX_LENGTH - 1);
        current_config.serial_number[SERIAL_NUM_MAX_LENGTH - 1] = '\0';

        std::cout << "Configuration: HW Rev = " << (int)current_config.hardware_revision
                  << ", Serial Number = " << current_config.serial_number << std::endl;
    }

    void tempMonitorTask() {
        if (new_adc_value_ready) {
            new_adc_value_ready = false;
            int16_t raw_adc_value = static_cast<int16_t>(latest_adc_value);

            float temperature_celsius;
            if (current_config.hardware_revision == HW_REV_A) {
                temperature_celsius = static_cast<float>(raw_adc_value);
            } else if (current_config.hardware_revision == HW_REV_B) {
                temperature_celsius = static_cast<float>(raw_adc_value) / 10.0f;
            } else {
                temperature_celsius = 0.0f;
                std::cout << "Error: Unknown hardware revision!" << std::endl;
            }

            std::cout << "Temperature: " << temperature_celsius << " °C" << std::endl;

            if (temperature_celsius >= TEMP_CRITICAL_LOW_THRESHOLD && temperature_celsius < TEMP_WARNING_THRESHOLD) {
                current_temp_state = TEMP_OK;
                std::cout << "Temperature State: OK" << std::endl;
            } else if (temperature_celsius >= TEMP_WARNING_THRESHOLD &&
                       temperature_celsius < TEMP_CRITICAL_HIGH_THRESHOLD) {
                current_temp_state = TEMP_WARNING;
                std::cout << "Temperature State: WARNING" << std::endl;
            } else if (temperature_celsius >= TEMP_CRITICAL_HIGH_THRESHOLD || temperature_celsius < TEMP_CRITICAL_LOW_THRESHOLD) {
                current_temp_state = TEMP_CRITICAL;
                std::cout << "Temperature State: CRITICAL" << std::endl;
            }

            ledControllerSetState(current_temp_state);
        }
    }

    void ledControllerSetState(TempState state) {
        switch (state) {
            case TEMP_OK:
                std::cout << "Green LED ON, Yellow LED OFF, Red LED OFF" << std::endl;
                break;
            case TEMP_WARNING:
                std::cout << "Green LED OFF, Yellow LED ON, Red LED OFF" << std::endl;
                break;
            case TEMP_CRITICAL:
                std::cout << "Green LED OFF, Yellow LED OFF, Red LED ON" << std::endl;
                break;
            default:
                std::cout << "All LEDs OFF (Invalid State)" << std::endl;
                break;
        }
    }

    void timerInterruptHandler() {
        adcStartConversion();
        latest_adc_value = adcReadRaw();
        new_adc_value_ready = true;
    }

    void run() {
        i2cInit(100000);
        adcInit();
        gpioInit(GREEN_LED_PIN);
        gpioInit(YELLOW_LED_PIN);
        gpioInit(RED_LED_PIN);

        configInit();

        for (int i = 0; i < 20; ++i) {
            timerInterruptHandler();
            tempMonitorTask();
            usleep(1000000);
        }

        std::cout << "Exiting simulation." << std::endl;
    }
};

int main() {
    TempSensorMonitoring system;
    system.run();
    return 0;
}
