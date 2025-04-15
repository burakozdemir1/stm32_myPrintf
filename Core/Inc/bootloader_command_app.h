/*
 * bootloader_command_app.h
 *
 *  Created on: Mar 15, 2025
 *      Author: burak
 */

#ifndef INC_BOOTLOADER_COMMAND_APP_H_
#define INC_BOOTLOADER_COMMAND_APP_H_

#include "main.h"

#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_flash_ex.h"

#define BL_VER	0x10		//	1.0

#define CRC_FAIL		1
#define CRC_SUCCESS	0

#define BL_ACK_VALUE    0xA5
#define BL_NACK_VALUE		0x7F

#define ADDR_VALID			0x00
#define ADDR_INVALID		0x01

#define SRAM_SIZE			16*1024
#define SRAM_END				(SRAM_BASE + SRAM_SIZE)
#define SRAM2_SIZE			16*1024
#define SRAM2_END				(SRAM2_BASE + SRAM2_SIZE)
#define BKPSRAM_SIZE		4*1024
#define BKPSRAM_END			(BKPSRAM_BASE + BKPSRAM_SIZE)

//#define INVALID_SECTOR 	0x04
#define INVALID_PAGE       0xFF
#define FLASH_START_ADDR   0x08000000UL

#define FLASH_TOTAL_SIZE   (128 * 1024UL)     // 128 KB toplam flash (örneğin STM32F072RB)
#define TOTAL_PAGES        (FLASH_TOTAL_SIZE / FLASH_PAGE_SIZE)

#define FLASH_PAGE_SIZE         0x800       // 2KB
#define PAGES_PER_SECTOR        2
#define SECTOR_SIZE             (FLASH_PAGE_SIZE * PAGES_PER_SECTOR)  // 4KB
#define FLASH_START_ADDR        0x08000000
#define TOTAL_SECTORS           32 / PAGES_PER_SECTOR  // 16

#define INVALID_SECTOR          0xFF



void bootloader_get_ver_cmd(uint8_t *bl_rx_data);
void bootloader_get_help_cmd(uint8_t *bl_rx_data);
void bootloader_get_cid_cmd(uint8_t *bl_rx_data);
void bootloader_get_rdp_cmd(uint8_t *bl_rx_data);
void bootloader_go_to_addr_cmd(uint8_t *bl_rx_data);
void bootloader_flash_erase_cmd(uint8_t *bl_rx_data);
void bootloader_mem_write_cmd(uint8_t* bl_rx_data);
void bootloader_handle_mem_write_cmd(uint8_t* bl_rx_data);
void bootloader_enable_read_write_protect_cmd(uint8_t* bl_rx_data);
void bootloader_read_sector_protection_status_cmd(uint8_t *bl_rx_data);
void bootloader_disable_read_write_protect_cmd(uint8_t *bl_rx_data);


/*												CRC Verify Function														*/
uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost);

void bootloader_send_ack(uint8_t followLength);
void bootloader_send_nack(void);

uint8_t bootloader_get_version(void);

void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len);

uint16_t get_mcu_chip_id(void);

uint8_t get_flash_rdp_level(void);

uint8_t bootloader_verify_address(uint32_t goAddress);

uint8_t execute_flash_erase(uint8_t sectorNumber, uint8_t numberOfSectors);

uint8_t execute_memory_write(uint8_t *Buffer, uint32_t memAddress, uint32_t len);

uint8_t configure_flash_sector_r_w_protection(uint8_t sector_details, uint8_t protection_mode,uint8_t enableOrDisable);

uint64_t read_OB_r_w_protection_status();


#endif /* INC_BOOTLOADER_COMMAND_APP_H_ */
