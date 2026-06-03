#ifndef FLASH_WRITER_H_
#define FLASH_WRITER_H_

#include "stdint.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0   ((uint32_t)0x08040000) /* Base address of Sector 0, 2  Kbytes */
#define ADDR_FLASH_SECTOR_1   ((uint32_t)0x08040800) /* Base address of Sector 1, 2  Kbytes */
#define ADDR_FLASH_SECTOR_2   ((uint32_t)0x08041000) /* Base address of Sector 2, 2  Kbytes */
#define ADDR_FLASH_SECTOR_3   ((uint32_t)0x08041800) /* Base address of Sector 3, 2  Kbytes */
#define ADDR_FLASH_SECTOR_4   ((uint32_t)0x08042000) /* Base address of Sector 4, 2  Kbytes */
#define ADDR_FLASH_SECTOR_5   ((uint32_t)0x08042800) /* Base address of Sector 5, 2 Kbytes */
#define ADDR_FLASH_SECTOR_6   ((uint32_t)0x08043000) /* Base address of Sector 6, 2 Kbytes */
#define ADDR_FLASH_SECTOR_7   ((uint32_t)0x08043800) /* Base address of Sector 7, 2 Kbytes */
#define ADDR_FLASH_SECTOR_MAX ((uint32_t)0x08044000)

void flash_erase_address(uint16_t page, uint16_t len);
int8_t flash_write_single_address(uint32_t start_address, uint64_t *buf, uint32_t len);
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
uint32_t get_sector(uint32_t address);
uint32_t get_next_flash_address(uint32_t address);
#endif
