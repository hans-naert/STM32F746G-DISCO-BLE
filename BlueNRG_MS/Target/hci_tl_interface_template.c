/**
  ******************************************************************************
  * @file    hci_tl_interface_template.c
  * @author  SRA Application Team
  * @brief   Function implementation for the STM32 BlueNRG HCI Transport Layer
  *          interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "hci_tl.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

/* Defines -------------------------------------------------------------------*/

#define HEADER_SIZE       5U
#define MAX_BUFFER_SIZE   255U
#define TIMEOUT_DURATION  15U
#define HCI_TL_SPI_CS_PORT CSN_GPIO_Port
#define HCI_TL_SPI_CS_PIN  CSN_Pin
#define HCI_TL_SPI_EXTI_PORT IRQ_GPIO_Port
#define HCI_TL_SPI_EXTI_PIN  IRQ_Pin
#define HCI_TL_RST_PORT    RST_GPIO_Port
#define HCI_TL_RST_PIN     RST_Pin
#define BSP_SPI2_SendRecv(tx_data,rx_data,size) HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, size, HAL_MAX_DELAY)
#define BSP_GetTick (int32_t (*)(void))HAL_GetTick

/* Private variables ---------------------------------------------------------*/
EXTI_HandleTypeDef hexti0;

/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 *
 * @param  buffer : Buffer where data from SPI are stored
 * @param  size   : Buffer size
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Receive(uint8_t* buffer, uint16_t size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_RESET);

  /* Read the header */
  BSP_SPI2_SendRecv(header_master, header_slave, HEADER_SIZE);

  if(header_slave[0] == 0x02)
  {
    /* device is ready */
    byte_count = (header_slave[4] << 8)| header_slave[3];

    if(byte_count > 0)
    {
      /* avoid to read more data than the size of the buffer */
      if (byte_count > size){
        byte_count = size;
      }

      for(len = 0; len < byte_count; len++)
      {
        BSP_SPI2_SendRecv(&char_ff, (uint8_t*)&read_char, 1);
        buffer[len] = read_char;
      }
    }
  }
  /* Release CS line */
  HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET);

#if PRINT_CSV_FORMAT
  if (len > 0) {
    print_csv_time();
    for (int i=0; i<len; i++) {
      PRINT_CSV(" %02x", buffer[i]);
    }
    PRINT_CSV("\n");
  }
#endif

  return len;
}


/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 *
 * @param  None
 * @retval int32_t: 1 if data are present, 0 otherwise
 */
static int32_t IsDataAvailable(void)
{
  return (HAL_GPIO_ReadPin(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN) == GPIO_PIN_SET);
}

/**
 * @brief  Writes data from local buffer to SPI.
 *
 * @param  buffer : data buffer to be written
 * @param  size   : size of first data buffer to be written
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Send(uint8_t* buffer, uint16_t size)
{
  int32_t result;

  uint8_t header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  static uint8_t read_char_buf[MAX_BUFFER_SIZE];
  uint32_t tickstart = HAL_GetTick();

  do
  {
    result = 0;

    /* CS reset */
    HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_RESET);

    /* Read header */
    BSP_SPI2_SendRecv(header_master, header_slave, HEADER_SIZE);

    if(header_slave[0] == 0x02)
    {
      /* SPI is ready */
      if(header_slave[1] >= size)
      {
        BSP_SPI2_SendRecv(buffer, read_char_buf, size);
      }
      else
      {
        /* Buffer is too small */
        result = -2;
      }
    } else {
      /* SPI is not ready */
      result = -1;
    }

    /* Release CS line */
    HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET);

    if((HAL_GetTick() - tickstart) > TIMEOUT_DURATION)
    {
      result = -3;
      break;
    }
  } while(result < 0);

  return result;
}

int32_t HCI_TL_SPI_Init()
{
  return 0;
}

int32_t  HCI_TL_SPI_DeInit()
{
  return 0;
}

int32_t  HCI_TL_SPI_Reset()
{
  // Deselect CS PIN for BlueNRG to avoid spurious commands
  HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  return 0;
}

void hci_tl_lowlevel_init(void)
{

  /* USER CODE BEGIN hci_tl_lowlevel_init 1 */
  /* Register IO bus services */
    tHciIO fops;

  /* Register IO bus services */
  fops.Init    = HCI_TL_SPI_Init;
  fops.DeInit  = HCI_TL_SPI_DeInit;
  fops.Send    = HCI_TL_SPI_Send;
  fops.Receive = HCI_TL_SPI_Receive;
  fops.Reset   = HCI_TL_SPI_Reset;
  fops.GetTick = BSP_GetTick;
  
    hci_register_io_bus (&fops);

  /* USER CODE END hci_tl_lowlevel_init 1 */

  /* USER CODE BEGIN hci_tl_lowlevel_init 2 */
  /* Register event irq handler */
  
  HAL_EXTI_GetHandle(&hexti0, EXTI_LINE_0);
  HAL_EXTI_RegisterCallback(&hexti0, HAL_EXTI_COMMON_CB_ID, hci_tl_lowlevel_isr);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);


  /* USER CODE END hci_tl_lowlevel_init 2 */

  /* USER CODE BEGIN hci_tl_lowlevel_init 3 */

  /* USER CODE END hci_tl_lowlevel_init 3 */
}

void hci_tl_lowlevel_isr(void)
{
  /* USER CODE BEGIN hci_tl_lowlevel_isr */
  /* Call hci_notify_asynch_evt() */
  while(IsDataAvailable())
  {
    if (hci_notify_asynch_evt(NULL))
    {
      return;
    }
  }

  /* USER CODE END hci_tl_lowlevel_isr */
}
