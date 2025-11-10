#pragma once

#include "drivers/spi.h"
#include "drivers/pwm.h"

// TLE9201 SPI interface for daisy chained chips
#define TLE9201_CS_PORT GPIOA
#define TLE9201_CS_PIN 10

// TLE9201 SPI commands per datasheet
#define TLE9201_RD_DIA    0x00  // Read Diagnosis Register
#define TLE9201_RES_DIA   0x80  // Reset Diagnosis Register  
#define TLE9201_RD_REV    0x20  // Read Device Revision
#define TLE9201_RD_CTRL   0x60  // Read Control Register
#define TLE9201_WR_CTRL   0xE0  // Write Control Register

// Implementation
void actuator_spi_init(void) {
    // Ensure SPI1 clock is enabled
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    // Hard reset SPI1 peripheral
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    
    // Configure SPI pins as alternate function
    set_gpio_mode(GPIOA, 5, MODE_ALTERNATE);  // SCK
    set_gpio_mode(GPIOA, 6, MODE_ALTERNATE);  // MISO  
    set_gpio_mode(GPIOA, 7, MODE_ALTERNATE);  // MOSI
    set_gpio_alternate(GPIOA, 5, GPIO_AF5_SPI1);
    set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
    set_gpio_alternate(GPIOA, 7, GPIO_AF5_SPI1);
    
    // Remove pullup to see if TLE9201 can drive SO pin at all
    set_gpio_pullup(GPIOA, 6, PULL_NONE);
    
    // CS pin as GPIO
    set_gpio_mode(TLE9201_CS_PORT, TLE9201_CS_PIN, MODE_OUTPUT);
    set_gpio_output(TLE9201_CS_PORT, TLE9201_CS_PIN, 1);
    
    // Configure SPI1 from scratch
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    
    // TLE9201 samples SI on falling edge of SCK = SPI Mode 1 (CPOL=0, CPHA=1)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t spi_transfer_byte(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void actuator_tle9201_read_daisy_chain(uint8_t cmd1, uint8_t cmd2, uint8_t *resp1, uint8_t *resp2) {
    set_gpio_output(TLE9201_CS_PORT, TLE9201_CS_PIN, 0);
    
    // Small setup delay
    for(volatile int i = 0; i < 100; i++);
    
    // Send commands to both chips (8-bit each)
    uint8_t r1 = spi_transfer_byte(cmd2);  // Command to chip 2
    uint8_t r2 = spi_transfer_byte(cmd1);  // Command to chip 1
    
    // Small hold delay
    for(volatile int i = 0; i < 100; i++);
    
    set_gpio_output(TLE9201_CS_PORT, TLE9201_CS_PIN, 1);
    
    *resp1 = r1;  // Response from chip 1
    *resp2 = r2;  // Response from chip 2
}

void actuator_tle9201_read_all_data(void) {
    // Reconfigure PA6 back to SPI MISO
    set_gpio_mode(GPIOA, 6, MODE_ALTERNATE);
    set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
    set_gpio_pullup(GPIOA, 6, PULL_UP);
    
    // Try actual SPI transaction
    set_gpio_output(TLE9201_CS_PORT, TLE9201_CS_PIN, 0);
    for(volatile int i = 0; i < 50; i++);
    
    uint8_t response = spi_transfer_byte(0x00);
    
    for(volatile int i = 0; i < 100; i++);
    set_gpio_output(TLE9201_CS_PORT, TLE9201_CS_PIN, 1);
    
    UNUSED(response);
}

