#include "usr_spi.h"
#include "string.h"

SPI_PROTOCAL_TypeDef __attribute((section(".RAM2"))) spi_protocals[CHIP_NUMBERS];

// the chip order on board: 3 1 4
uint16_t CS_GPIOS[3] = {SPI3_CS_Pin, SPI1_CS_Pin, SPI4_CS_Pin};

void init_spi_protocals(void) {
  for (uint8_t i = 0; i < CHIP_NUMBERS; i++) {
    // size of cmd is much bigger than data
    spi_protocals[i].SPI_CS_Pin = CS_GPIOS[i];
    memset(&(spi_protocals[i].spi_cmd_u), 0, sizeof(SPI_CMD_U));
    memset(&(spi_protocals[i].spi_data_u), 0, sizeof(SPI_CMD_U));
  }
  spi_protocals[0].SPI_CS_Port = SPI3_CS_GPIO_Port;
  spi_protocals[1].SPI_CS_Port = SPI1_CS_GPIO_Port;
  spi_protocals[2].SPI_CS_Port = SPI4_CS_GPIO_Port;

  spi_protocals[0].Spi_Channel = &hspi3;
  spi_protocals[1].Spi_Channel = &hspi1;
  spi_protocals[2].Spi_Channel = &hspi4;
}

SPI_PROTOCAL_TypeDef* get_spi_protocal(uint8_t i) {
  return &spi_protocals[i];
}