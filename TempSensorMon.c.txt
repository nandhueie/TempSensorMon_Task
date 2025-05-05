#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
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

#define ADC_CHANNEL 0

#define TIMER_ID 0
#define TIMER_SIGNAL SIGALRM // Use SIGALRM for timer simulation


typedef struct {
    uint8_t hardware_revision;
    char serial_number[SERIAL_NUM_MAX_LENGTH];
} system_config_t;


typedef enum {
    TEMP_OK,
    TEMP_WARNING,
    TEMP_CRITICAL
} temp_state_t;

system_config_t current_config;
volatile uint16_t latest_adc_value;
volatile bool new_adc_value_ready = false;
temp_state_t current_temp_state = TEMP_OK;
void (*timer_interrupt_handler_ptr)(int) = NULL; // ISR function pointer


void config_init(void);
void temp_monitor_task(void);
void led_controller_set_state(temp_state_t state);
void timer_interrupt_handler(int sig);


void gpio_init(uint32_t pin) {
    printf("GPIO Pin %u Initialized as output \n", pin);
}

void gpio_set(uint32_t pin, uint8_t state) {
    printf("GPIO Pin %u set \n", pin);
}

void timer_enable_interrupt(void) {
    printf("Timer interrupt enabled\n");
}

void timer_clear_interrupt_flag(void) {
    //printf("Timer interrupt flag cleared\n");
}

bool timer_interrupt_flag_is_set(void) {
    return false;
}

void interrupt_enable(void) {
    printf("Global interrupts enabled\n");
}

void interrupt_disable(void) {
    printf("Global interrupts disabled\n");
}

void interrupt_register_handler(int sig_num, void (*handler)(int)) {
    printf("Interrupt handler registered for signal \n");
    signal(sig_num, handler);
    timer_interrupt_handler_ptr = handler;
}

void i2c_init_drv(uint32_t speed) {
    printf("I2C Initialized at %u Hz\n", speed);
}

uint8_t i2c_read_byte(uint8_t device_address, uint8_t memory_address) {
    printf("I2C Read Byte from 0x%02X, Address 0x%02X\n", device_address, memory_address);
    return HW_REV;
}

void i2c_read_block(uint8_t device_address, uint8_t memory_address, uint8_t *data, uint16_t length) {
    printf("I2C Read Block from 0x%02X, Address 0x%02X, Length %u\n",
           device_address, memory_address, length);
    snprintf((char *)data, length, "ABC1234");
}


void adc_init_drv(void) {
    printf("ADC Initialized\n");
}

void adc_start_conversion(void) {
    //printf("ADC conversion started\n");
}

bool adc_conversion_complete(void) {
    return true;
}

uint16_t adc_read_raw(void) {
    static int16_t adc_value = -200;
    adc_value += 80;
    if (adc_value > 1500) {
        adc_value = -200;
    }
    return (uint16_t)adc_value;
}

void config_init(void) {

    current_config.hardware_revision = i2c_read_byte(EEPROM_ADDRESS, HW_REV);

    i2c_read_block(EEPROM_ADDRESS, SERIAL_NUM_ADDR,
                       (uint8_t *)current_config.serial_number,
                       SERIAL_NUM_MAX_LENGTH - 1);

    current_config.serial_number[SERIAL_NUM_MAX_LENGTH - 1] = '\0';

    printf("Configuration: HW Rev = %d, Serial Number = %s\n", current_config.hardware_revision, current_config.serial_number);
}


void temp_monitor_task(void) {
    if (new_adc_value_ready) {
        new_adc_value_ready = false;
        int16_t raw_adc_value = (int16_t)latest_adc_value;

        float temperature_celsius;
        if (current_config.hardware_revision == HW_REV_A) {
            temperature_celsius = (float)raw_adc_value;
        } else if (current_config.hardware_revision == HW_REV_B) {
            temperature_celsius = (float)raw_adc_value / 10.0f;
        } else {
            temperature_celsius = 0.0f;
            printf("Error: Unknown hardware revision!\n");
        }
        printf("**************************\n");
        printf("Raw ADC Value: %d\n", raw_adc_value);
        printf("Temperature: %.2f °C\n", temperature_celsius);
        printf("**************************\n");

        if (temperature_celsius >= TEMP_CRITICAL_LOW_THRESHOLD && temperature_celsius < TEMP_WARNING_THRESHOLD) {
            current_temp_state = TEMP_OK;
            printf("Temperature State: OK\n");
        } else if (temperature_celsius >= TEMP_WARNING_THRESHOLD &&
                   temperature_celsius < TEMP_CRITICAL_HIGH_THRESHOLD) {
            current_temp_state = TEMP_WARNING;
            printf("Temperature State: WARNING\n");
        } else if (temperature_celsius >= TEMP_CRITICAL_HIGH_THRESHOLD || temperature_celsius < TEMP_CRITICAL_LOW_THRESHOLD) {
            current_temp_state = TEMP_CRITICAL;
            printf("Temperature State: CRITICAL\n");
        }

        led_controller_set_state(current_temp_state);
    }
}

void led_controller_set_state(temp_state_t state) {
    switch (state) {
        case TEMP_OK:
            printf("Green LED ON, Yellow LED OFF, Red LED OFF\n\n");
            break;
        case TEMP_WARNING:
            printf("Green LED OFF, Yellow LED ON, Red LED OFF\n\n");
            break;
        case TEMP_CRITICAL:
            printf("Green LED OFF, Yellow LED OFF, Red LED ON\n\n");
            break;
        default:
            printf("All LEDs OFF (Invalid State)\n\n");
            break;
    }
}

void timer_interrupt_handler(int sig) {

    adc_start_conversion();

    latest_adc_value = adc_read_raw();
    new_adc_value_ready = true;

    timer_clear_interrupt_flag(); 
    //printf("Timer interrupt occurred (signal %d)\n", sig);
}


int main(void) {
    // Initialize system
    interrupt_disable();

    // Initialize hardware modules
    i2c_init_drv(100000);
    adc_init_drv();
    gpio_init(GREEN_LED_PIN);
    gpio_init(YELLOW_LED_PIN);
    gpio_init(RED_LED_PIN);

    config_init();

    interrupt_register_handler(TIMER_SIGNAL, timer_interrupt_handler);
    timer_enable_interrupt();
    interrupt_enable();

    for (int i = 0; i < 20; ++i) {
        raise(TIMER_SIGNAL);

        temp_monitor_task();
        usleep(1000000); // 100microseconds delay
    }

    printf("Exiting simulation.\n");
    return 0;
}

