/*
 * bootloader_command_app.c
 *
 *  Created on: Mar 15, 2025
 *      Author: burak
 */


#include "bootloader_command_app.h"
#include <string.h>
extern uint8_t supported_commands[] ;


//
//void bootloader_get_ver_cmd(uint8_t *bl_rx_data)
//{
//	uint8_t bl_version=0;
//
//	printMessage("BL_DEBUG_MSG: Bootloader Get Version Command\n");
//	uint32_t command_packet_length = bl_rx_data[0] + 1 ;
//	uint32_t host_crc = *((uint32_t *)(bl_rx_data + command_packet_length -4));
//
//	if(!bootloader_verify_crc(&bl_rx_data[0],command_packet_length-4,host_crc)) // -1-
//	{
//		printMessage("BL_DEBUG_MSG: Checksum Success\n");
//		bootloader_send_ack( bl_rx_data[0] , 1); // ack bilgisi gönderirken sonrasında kaç byte lık veri göndereceğini gönder
//		bl_version = bootloader_get_version() ;
//		printMessage("BL_DEBUG_MSG: BL VERSION : %d\n",bl_version);
//		bootloader_uart_write_data(&bl_version,1);
//
//		/*
//
//		 1 de yaptığımız iş şu ;
//
//		 Bizim BL_GET VER komutumuz şu şekilde ;
//
//		 	 Length to Follow(1)  |  Command Code(1)  |  CRC (4)
//		 ////////////////////////   ///////////////
//		 	bu kısım ile 				bu kısmın
//		 			CRC sini hesapladık
//		 					ve
//		 			CRC(4) kısmının crc si ile
//		 				uyuşup uyuşmadığını
//		 				   kontrol ettik.
//		 */
//
//	}
//	else
//	{
//		printMessage("BL_DEBUG_MSG: Checksum Fail!\n");
//		bootloader_send_nack();
//	}
//}
//
//
//uint8_t bootloader_verify_crc(uint8_t *Buffer,uint32_t len,uint32_t crcHost)
//{
//	uint32_t crcValue = 0xff; // crc değerlerimi içine almak için
//	uint32_t data = 0;        // dizinin her bir elemanını buna atarım
//	for(uint32_t i = 0 ; i<len ; i++)
//	{
//		data = Buffer[i];
//		crcValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
//		/*
//		 HAL_CRC_Accumulate -> her gelen verinin içindeki crc bilgilerini çıkarıp crcValue değerine ekler
//		 gelen tümm Buffer verilerinin içindeki crc değerlerini alıp crcValue ye atarım
//		 */
//	}
//
//	__HAL_CRC_DR_RESET(&hcrc);
//
//	if(crcValue==crcHost)
//	{
//		return CRC_SUCCESS ;
//	}
//
//	return CRC_FAIL ;
//}
//
//
//void bootloader_send_ack(uint8_t commandCode , uint8_t followLength)
//{
//	// ack bilgisi gönderirken sonrasında kaç byte lık veri göndereceğini gönder
//	// ack bilgisi her zaman 2 byte olarak gönderilir
//
//	uint8_t ackBuffer[2];
//	ackBuffer[0] = BL_ACK_VALUE ;
//	ackBuffer[1] = followLength ; // ack bilgisi gönderirken sonrasında kaç byte lık veri göndereceğini gönder
//
//	HAL_UART_Transmit(&huart1, ackBuffer, 2, HAL_MAX_DELAY);
//
//
//
//
//}
//void bootloader_send_nack(void)
//{
//	uint8_t nackValue = BL_NACK_VALUE;
//	HAL_UART_Transmit(&huart1, &nackValue, 1, HAL_MAX_DELAY);
//}
//
//void bootloader_uart_write_data(uint8_t *buffer,uint32_t len)
//{
//	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
//}
//
//uint8_t bootloader_get_version(void)
//{
//	return BL_VER ;
//}

//
//void bootloader_get_ver_cmd(uint8_t *bl_rx_data)
//{
//	uint8_t bl_Version = 0;
//
//	printMessage("BL_DEBUG_MSG: Bootloaer_Get_Ver_Cmd\n");
//
//	uint32_t command_packet_length = bl_rx_data[0] + 1;
//
//	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_length - 4));
//
//	// crc control
//	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_length - 4, host_crc))
//	{
//		printMessage("BL_DEBUG_MSG: Checksum success\n");
//	//	bootloader_send_ack(1);
//		bl_Version = bootloader_get_version();
//		printMessage("BL_DEBUG_MSG: BL_VER : %d %#x  \n", bl_Version, bl_Version);
//		bootloader_uart_write_data(&bl_Version, 1);
//	}
//	else
//	{
//		printMessage("BL_DEBUG_MSG: Checsum fail \n");
//		bootloader_send_nack();
//	}
//}
//void bootloader_get_help_cmd(uint8_t *bl_rx_data)
//{
//	printMessage("BL_DEBUG_MSG: bootloader_get_help_cmd\n");
//
//	uint32_t commant_packet_len = bl_rx_data[0] + 1 ;
//	uint32_t host_crc = *((uint32_t*)(bl_rx_data + commant_packet_len - 4));
//
//	if(!bootloader_verify_crc(&bl_rx_data[0], commant_packet_len - 4, host_crc))
//	{
//		printMessage("BL_DEBUG_MSG: Checksum success\n");
//		bootloader_send_ack(bl_rx_data[0],strlen(supported_commands));
//
//		bootloader_uart_write_data(supported_commands, strlen(supported_commands));
//
//	}
//	else
//	{
//		printMessage("BL_DEBUG_MSG: Checsum fail \n");
//		bootloader_send_nack();
//	}
//}
//
//
//
//
//void bootloader_disable_read_write_protect_cmd(uint8_t *bl_rx_data)
//{
//	uint8_t status = 0;
//
//	printMessage("BL_DEBUG_MSG: bootloader_disable_read_write_protect_cmd \n");
//
//	uint32_t command_packet_len = bl_rx_data[0] + 1;
//
//	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));
//
//	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
//	{
//		printMessage("BL_DEBUG_MSG: Checksum success \n");
////	bootloader_send_ack(1);
//
//		status = configure_flash_sector_r_w_protection(0, 0, 1);
//
//		printMessage("BL_DEBUG_MSG: Status: %d", status);
//
//		bootloader_uart_write_data(&status, 1);
//	}
//	else
//	{
//		printMessage("BL_DEBUG_MSG: Checksum fail \n");
//		bootloader_send_nack();
//	}
//}
//
//void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len)
//{
//	HAL_UART_Transmit(&huart1, Buffer, len, HAL_MAX_DELAY);
//}
//
//uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost)
//{
//	uint32_t crcValue = 0xFF;
//	uint32_t data = 0;
//
//	for(uint32_t i = 0; i < len; i++)
//	{
//			data = Buffer[i];
//			crcValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
//	}
//
//	__HAL_CRC_DR_RESET(&hcrc);
//
//	if(crcValue == crcHost)
//	{
//		return CRC_SUCCESS;
//	}
//
//	return CRC_FAIL;
//}
//
//void bootloader_send_ack(uint8_t commandCode,uint8_t followLength)
//{
//	uint8_t ackBuffer[2];
//	ackBuffer[0] = BL_ACK_VALUE;
//	ackBuffer[1] = followLength;
//
//	HAL_UART_Transmit(&huart1, ackBuffer, 2, HAL_MAX_DELAY);
//}
//
//void bootloader_send_nack()
//{
//	uint8_t nackValue = BL_NACK_VALUE;
//	HAL_UART_Transmit(&huart1, &nackValue, 1, HAL_MAX_DELAY);
//}
//
//uint8_t bootloader_get_version(void)
//{
//	return BL_VER;
//}
#include "bootloader_command_app.h"

extern uint8_t supported_commands[];

void bootloader_get_ver_cmd(uint8_t *bl_rx_data)
{
	uint8_t bl_Version = 0;

	printMessage("BL_DEBUG_MSG: Bootloaer_Get_Ver_Cmd\n");

	uint32_t command_packet_length = bl_rx_data[0] + 1;

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_length - 4, sizeof(host_crc));

	// crc control
	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_length - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success\n");
		bootloader_send_ack(1);
		bl_Version = bootloader_get_version();
		printMessage("BL_DEBUG_MSG: BL_VER : %d %#x  \n", bl_Version, bl_Version);
		bootloader_uart_write_data(&bl_Version, 1);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checsum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_get_help_cmd(uint8_t *bl_rx_data)
{
	printMessage("BL_DEBUG_MSG: bootloader_get_help_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

//	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_len - 4, sizeof(host_crc));


	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum succes \n");
		bootloader_send_ack(strlen(supported_commands));
		bootloader_uart_write_data(supported_commands, strlen(supported_commands));
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}

}

void bootloader_get_cid_cmd(uint8_t *bl_rx_data)
{
	uint16_t cID = 0;

	printMessage("BL_DEBUG_MSG: bootloader_get_cid_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_len - 4, sizeof(host_crc));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum succes \n");
		bootloader_send_ack(2);
		cID = get_mcu_chip_id();
		printMessage("BL_DUBEG_MSG: STM32F072 Chip Id: %d %#x \n", cID, cID);
		bootloader_uart_write_data((uint8_t*)&cID, 2);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_get_rdp_cmd(uint8_t *bl_rx_data)
{
	uint8_t rdpLevel = 0;

	printMessage("BL_DEBUG_MSG: bootloader_get_rdp_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_len - 4, sizeof(host_crc));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum succes \n");
		bootloader_send_ack(1);
		rdpLevel = get_flash_rdp_level();
		printMessage("BL_DEBUG_MSG: STM32F072 RDP Level: %d %#x \n", rdpLevel, rdpLevel);
		bootloader_uart_write_data(&rdpLevel, 1);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}

}

void bootloader_go_to_addr_cmd(uint8_t *bl_rx_data)
{
	uint32_t go_to_address = 0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;

	printMessage("BL_DEBUG_MSG: bootlodaer_go_to_addr_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_len - 4, sizeof(host_crc));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum succes \n");
		bootloader_send_ack(1);

		memcpy(&go_to_address, &bl_rx_data[2], sizeof(go_to_address));
		printMessage("BL_DEBUG_MSG: GO Addr: %#x \n", go_to_address);

		if( bootloader_verify_address(go_to_address) == ADDR_VALID )
		{
			bootloader_uart_write_data(&addr_valid, 1);

			go_to_address += 1; 		// T Bit = 1

			void (*lets_go_to_address)(void) = (void*) go_to_address;

			printMessage("BL_DEBUG_MSG: Going to Address \n");

			lets_go_to_address();
		}
		else
		{
			printMessage("BL_DEBUG_MSG: Go Address Invalid \n");
			bootloader_uart_write_data(&addr_invalid, 1);
		}
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_flash_erase_cmd(uint8_t *bl_rx_data)
{
	uint8_t eraseStatus = 0;

	printMessage("BL_DEBUG_MSG: bootloader_flash_erase_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = 0;
	memcpy(&host_crc, bl_rx_data + command_packet_len - 4, sizeof(host_crc));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success \n");
		bootloader_send_ack(1);
		printMessage("BL_DEBUG_MSG: Initial Sector: %d Nubmer Of Secotrs: %d \n", bl_rx_data[2], bl_rx_data[3]);

		eraseStatus = execute_flash_erase(bl_rx_data[2], bl_rx_data[3]);

		printMessage("BL_DEBUG_MSG: Flash Erase Status : %d \n", eraseStatus);
		bootloader_uart_write_data(&eraseStatus, 1);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_mem_write_cmd(uint8_t* bl_rx_data)
{
	uint8_t addrValid = ADDR_VALID;
	uint8_t writeStatus = 0x00;
	uint8_t checkSum = 0;
	uint8_t length = 0;

	length = bl_rx_data[0];

	uint8_t payloadLength = bl_rx_data[6];

//	uint32_t memAddress = *((uint32_t *) ( &bl_rx_data[2]) );
	uint32_t memAddress;

	memcpy(&memAddress, &bl_rx_data[2], sizeof(memAddress));
	checkSum = bl_rx_data[length];

	printMessage("BL_DEBUG_MSG: bootloader_mem_write_cmd \n");

//	uint32_t command_packet_len = bl_rx_data[0] + 1;

//	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));
	uint32_t host_crc = 0;
	uint32_t command_packet_len = bl_rx_data[0] + 1;

	memcpy(&host_crc, &bl_rx_data[command_packet_len - 4], sizeof(uint32_t));


	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success \n");
		bootloader_send_ack(1);

		printMessage("BL_DEBUG_MSG: Memory Write Address: %#x \n", memAddress);

		if(bootloader_verify_address(memAddress) == ADDR_VALID)
		{
			printMessage("BL_DEBUG_MSG: Valid Memory Write Address \n");

			writeStatus = execute_memory_write(&bl_rx_data[7], memAddress, payloadLength);

			bootloader_uart_write_data(&writeStatus, 1);
		}
		else
		{
			printMessage("BL_DEBUG_MSG: Invalid Memory Write Address \n");
			writeStatus = ADDR_INVALID;
			bootloader_uart_write_data(&writeStatus, 1);
		}
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_enable_read_write_protect_cmd(uint8_t* bl_rx_data)
{
	uint8_t status = 0;

	printMessage("BL_DEBUG_MSG: bootloader_enable_read_write_protect_cmd \n");

	uint32_t host_crc = 0;
	uint32_t command_packet_len = bl_rx_data[0] + 1;

	memcpy(&host_crc, &bl_rx_data[command_packet_len - 4], sizeof(uint32_t));


	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success \n");
		bootloader_send_ack(1);

		status = configure_flash_sector_r_w_protection(bl_rx_data[2], bl_rx_data[3], 0);

		printMessage("BL_DEBUG_MSG: Status: %d\n", status);

		bootloader_uart_write_data(&status, 1);
		HAL_FLASH_OB_Launch(); //  yaptığın option byte (OB) değişikliklerini etkinleştirir ve mikrodenetleyiciyi resetler.
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_read_sector_protection_status_cmd(uint8_t *bl_rx_data)
{
	uint16_t status = 0;

	printMessage("BL_DEBUG_MSG: bootloader_read_sector_protection_status_cmd \n");

	uint32_t host_crc = 0;
	uint32_t command_packet_len = bl_rx_data[0] + 1;

	memcpy(&host_crc, &bl_rx_data[command_packet_len - 4], sizeof(uint32_t));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success \n");
		bootloader_send_ack(1);

		status = read_OB_r_w_protection_status();

		printMessage("BL_DEBUG_MSG: nWRP status: %#\n", status);
		bootloader_uart_write_data((uint8_t*)&status, 2);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}

}

void bootloader_disable_read_write_protect_cmd(uint8_t *bl_rx_data)
{
	uint8_t status = 0;

	printMessage("BL_DEBUG_MSG: bootloader_disable_read_write_protect_cmd \n");

	uint32_t host_crc = 0;
	uint32_t command_packet_len = bl_rx_data[0] + 1;

	memcpy(&host_crc, &bl_rx_data[command_packet_len - 4], sizeof(uint32_t));

	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("BL_DEBUG_MSG: Checksum success \n");
		bootloader_send_ack(1);

		status = configure_flash_sector_r_w_protection(0, 0, 1);

		printMessage("BL_DEBUG_MSG: Status: %d", status);

		bootloader_uart_write_data(&status, 1);
	}
	else
	{
		printMessage("BL_DEBUG_MSG: Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len)
{
	HAL_UART_Transmit(&huart1, Buffer, len, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost)
{
	uint32_t crcValue = 0xFF;
	uint32_t data = 0;

	for(uint32_t i = 0; i < len; i++)
	{
			data = Buffer[i];
			crcValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
	}

	__HAL_CRC_DR_RESET(&hcrc);

	if(crcValue == crcHost)
	{
		return CRC_SUCCESS;
	}

	return CRC_FAIL;
}

void bootloader_send_ack(uint8_t followLength)
{
	uint8_t ackBuffer[2];
	ackBuffer[0] = BL_ACK_VALUE;
	ackBuffer[1] = followLength;

	HAL_UART_Transmit(&huart1, ackBuffer, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack()
{
	uint8_t nackValue = BL_NACK_VALUE;
	HAL_UART_Transmit(&huart1, &nackValue, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_get_version(void)
{
	return BL_VER;
}

uint16_t get_mcu_chip_id(void)
{
	uint16_t cID;
	cID = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cID;
}

uint8_t get_flash_rdp_level(void)
{
	uint8_t rdp_level = 0;

#if	0

	volatile uint32_t *OB_Addr = (uint32_t*) 0x1FFFC000;
	rdp_level = (uint8_t)(*OB_Addr >> 8);

#else

	FLASH_OBProgramInitTypeDef OB_InitStruct;
	HAL_FLASHEx_OBGetConfig(&OB_InitStruct);
	rdp_level = (uint8_t) OB_InitStruct.RDPLevel;

#endif

	return rdp_level;
}

uint8_t bootloader_verify_address(uint32_t goAddress)
{
	if(goAddress >= FLASH_BASE && goAddress <= FLASH_BANK1_END)
		return ADDR_VALID;
	else if(goAddress >= SRAM_BASE && goAddress <= SRAM_END)
		return ADDR_VALID;
//	else if(goAddress >= SRAM2_BASE && goAddress <= SRAM2_END)
//		return ADDR_VALID;
//	else if(goAddress >= BKPSRAM_BASE && goAddress <= BKPSRAM_END)
//		return ADDR_VALID;
	else
		return ADDR_INVALID;
}

uint8_t execute_flash_erase(uint8_t sectorNumber, uint8_t numberOfSectors)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    uint32_t pageError = 0;

    if (sectorNumber >= TOTAL_SECTORS) {
        return INVALID_SECTOR;
    }

    uint32_t sectorBaseAddress = FLASH_START_ADDR + (sectorNumber * SECTOR_SIZE);

    HAL_FLASH_Unlock();

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = sectorBaseAddress;
    eraseInitStruct.NbPages = numberOfSectors * PAGES_PER_SECTOR; // çünkü 1 sektör = 2 page

    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        return (uint8_t)status;
    }

    return HAL_OK;
}

uint8_t execute_memory_write(uint8_t *Buffer, uint32_t memAddress, uint32_t len) // buffer ın içeriğini flash adresine yazar
{

//FIXME: 8.sektöre denk gelir	0x08008000
    uint8_t status = HAL_OK;
    uint32_t data;

    HAL_FLASH_Unlock();

    for(uint32_t i = 0; i < len; i += 4)
    {
        // 4 byte’lık veri oluştur (little-endian)
        data = 0xFFFFFFFF; // default boş veri (eğer len % 4 != 0 ise eksikleri FF ile doldurur)
        data  = Buffer[i];
        if (i+1 < len) data |= (Buffer[i+1] << 8);
        if (i+2 < len) data |= (Buffer[i+2] << 16);
        if (i+3 < len) data |= (Buffer[i+3] << 24);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, memAddress + i, data);
        if(status != HAL_OK)
            break; // hata olursa döngüden çık
    }

    HAL_FLASH_Lock();

    return status;
}

uint8_t configure_flash_sector_r_w_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t enableOrDisable)
{






    FLASH_OBProgramInitTypeDef obInit;
    uint32_t wrp_pages = 0;
    uint8_t status = 0;


	if(enableOrDisable)
	{
	    HAL_FLASH_Unlock();
	    HAL_FLASH_OB_Unlock();


	    obInit.OptionType = OPTIONBYTE_WRP;
	    obInit.WRPPage = 0xFFFF;
	    obInit.WRPState = OB_WRPSTATE_DISABLE;

	    if (HAL_FLASHEx_OBProgram(&obInit) != HAL_OK)
	        status = 2;


	    HAL_FLASH_OB_Lock();
	    HAL_FLASH_Lock();

	    return status;
	}


    if (sector_details > 63 || protection_mode != 1)
        status = 1;
    else
    {
        for (uint8_t i = 0; i <= sector_details; i++)
            wrp_pages |= (1U << i);

        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();

        obInit.OptionType = OPTIONBYTE_WRP;
        obInit.WRPPage = wrp_pages;
        obInit.WRPState = (enableOrDisable == 0) ? OB_WRPSTATE_ENABLE : OB_WRPSTATE_DISABLE;

        if (HAL_FLASHEx_OBProgram(&obInit) != HAL_OK)
            status = 3;

        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
    }


    return status;

//	else if(protection_mode == 2) // read / write protection
//	{
//		HAL_FLASH_OB_Unlock();
//
//		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
//
//		*pOPTCR &= ~(0xFF << 16);					// write protecton all sector
//		*pOPTCR |= (sector_details << 16);
//
//		*pOPTCR |= (0xFF << 8);						// read protection all sector
//
//		*pOPTCR |= (1 << 1);
//
//		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
//
//		HAL_FLASH_OB_Lock();
//	}

	//
	//	if(enableOrDisable)
	//	{
	//		HAL_FLASH_OB_Unlock();
	//
	//		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	//
	//		*pOPTCR |= (0xFF << 16);
	//
	//		*pOPTCR |= (1 << 1);
	//
	//		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	//
	//		HAL_FLASH_OB_Lock();
	//
	//		return 0;
	//	}

//	return 0;
}

uint64_t read_OB_r_w_protection_status(void)
{
    FLASH_OBProgramInitTypeDef obInit;
    uint64_t page_status = 0;

    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBGetConfig(&obInit);
    HAL_FLASH_OB_Lock();

    uint32_t wrp_pages = obInit.WRPPage;

    for (uint8_t page = 0; page < 64; page++)
    {
        // Bit 0: korumalı, Bit 1: açık
        if (((wrp_pages >> page) & 0x1) == 0)
        {
            page_status |= ((uint64_t)1 << page);  // page korumalı
        }
    }

    return page_status;
}

