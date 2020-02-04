/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Server/Src/ethernetif.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file implements Ethernet network interface drivers for lwIP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "lwip/opt.h"
#include "lwip/lwip_timers.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "enc28j60.h"
#include <string.h>
#include "stm32f1xx_hal_spi.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 'e'
#define IFNAME1 'n'

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ENC_HandleTypeDef EncHandle;
extern SPI_HandleTypeDef hspi2;
#ifdef USE_PROTOTHREADS
static struct pt transmit_pt;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = 0x00000000U;
  hspi2.Init.CRCCalculation = 0x00000000U;
  HAL_SPI_Init(&hspi2);

}


/*******************************************************************************
                       ENC28J60 MSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the ENC28J60 MSP.
  * @param  heth: ENC28J60 handle
  * @retval None
  */
void ENC_MSPInit(ENC_HandleTypeDef *heth)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* ENC28J60 pins configuration ************************************************/
  /*
        ENC28J60_CS ----------------------> PB12
        ENC28J60_INT ---------------------> PD8
  */

  /*Configure GPIO pins : PB12 */
  GPIO_InitStructure.Pin = GPIO_PIN_12;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Deselect ENC28J60 module */
  HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_SET);

#ifdef ENC28J60_INTERRUPT
  /*Configure GPIO pin : PD8 */
  GPIO_InitStructure.Pin = GPIO_PIN_8;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif /* ENC28J60_INTERRUPT */

  /* Initialize SPI */
  MX_SPI2_Init();

#ifdef ENC28J60_INTERRUPT
  /* EXTI interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif /* ENC28J60_INTERRUPT */

}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static void low_level_init(struct netif *netif)
{
  //uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };

  /* Initialize transmit protothread */
#ifdef USE_PROTOTHREADS
  PT_INIT(&transmit_pt);
#endif

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;

  EncHandle.Init.MACAddr = netif->hwaddr;
  EncHandle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
  EncHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  EncHandle.Init.InterruptEnableBits =  EIE_LINKIE;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  ENC_MSPInit(&EncHandle);
  /* Set netif link flag */
//  netif->flags |= NETIF_FLAG_LINK_UP;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* Start the EN28J60 module */
  if (ENC_Start(&EncHandle)) {
    /* Set the MAC address */
    ENC_SetMacAddr(&EncHandle);

    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;
  }
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    /* TODO use netif to check if we are the right ethernet interface */
  err_t errval;
  struct pbuf *q;
  uint32_t framelength = 0;

#ifdef USE_PROTOTHREADS
if (EncHandle.transmitLength != 0) {
     while (PT_SCHEDULE(ENC_Transmit(&transmit_pt, &EncHandle))) {
         /* Wait for end of previous transmission */
     }
  }
#endif

  /* Prepare ENC28J60 Tx buffer */
  errval = ENC_RestoreTXBuffer(&EncHandle, p->tot_len);
  if (errval != ERR_OK) {
      return errval;
  }

  /* copy frame from pbufs to driver buffers and send packet */
  for(q = p; q != NULL; q = q->next) {
    ENC_WriteBuffer(q->payload, q->len);
    framelength += q->len;
  }

  if (framelength != p->tot_len) {
     return ERR_BUF;
  }

  EncHandle.transmitLength = p->tot_len;
  /* Actual transmission is triggered in main loop */

  return ERR_OK;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  struct pbuf *q;
  uint16_t len;
  uint8_t *buffer;
  uint32_t bufferoffset = 0;

  if (!ENC_GetReceivedFrame(&EncHandle)) {
    return NULL;
  }

  /* Obtain the size of the packet and put it into the "len" variable. */
  len = EncHandle.RxFrameInfos.length;
  buffer = (uint8_t *)EncHandle.RxFrameInfos.buffer;

  if (len > 0)
  {
    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  }

  if (p != NULL)
  {
    bufferoffset = 0;

    for(q = p; q != NULL; q = q->next)
    {
      /* Copy data in pbuf */
      memcpy( (uint8_t*)((uint8_t*)q->payload), (uint8_t*)((uint8_t*)buffer + bufferoffset), q->len);
      bufferoffset = bufferoffset + q->len;
    }
  }

  return p;
}

/**
  * @brief This function should be called when a packet is ready to be read
  * from the interface. It uses the function low_level_input() that
  * should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{

  err_t err;
  struct pbuf *p;
  /* move received packet into a new pbuf */
  p = low_level_input(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);

  if (err != ERR_OK)
  {
    pbuf_free(p);
    p = NULL;
  }
}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "stm32idisco";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  Returns the current time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/**
  * @brief  This function sets the netif link status.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_set_link(struct netif *netif)
{
    /* Handle ENC28J60 interrupt */
    ENC_IRQHandler(&EncHandle);

    /* Check whether the link is up or down*/
    if(((EncHandle.LinkStatus) & PHSTAT2_LSTAT)!= 0) {
      netif_set_link_up(netif);
    } else {
      netif_set_link_down(netif);
    }

    /* Reenable interrupts */
    ENC_EnableInterrupts(EIE_INTIE);
}

/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
  if(netif_is_link_up(netif)) {
      /* Restart the EN28J60 module */
      low_level_init(netif);
  }

  ethernetif_notify_conn_changed(netif);
}

/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file
            when the callback is needed,
  */
}

/**
 * Implement actual transmission triggering
 */
void ethernet_transmit(void) {
#ifdef USE_PROTOTHREADS
    ENC_Transmit(&transmit_pt, &EncHandle);
#else
    ENC_Transmit(&EncHandle);
#endif
}

/**
  * Implement SPI single byte send and receive.
  * The ENC28J60 slave SPI must already be selected and wont be deselected after transmission
  * Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

uint8_t ENC_SPI_SendWithoutSelection(uint8_t command)
{
    HAL_SPI_TransmitReceive(&hspi2, &command, &command, 1, 1000);
    return command;
}

/**
  * Implement SPI single byte send and receive. Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

uint8_t ENC_SPI_Send(uint8_t command)
{
    /* Select ENC28J60 module */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_RESET);
    up_udelay(1);

    HAL_SPI_TransmitReceive(&hspi2, &command, &command, 1, 1000);

    /* De-select ENC28J60 module */
    HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_SET);
    up_udelay(1);

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    return command;
}

/**
  * Implement SPI buffer send and receive. Must be provided by user code
  * param  master2slave: data to be sent from host to ENC28J60, can be NULL if we only want to receive data from slave
  * param  slave2master: answer from ENC28J60 to host, can be NULL if we only want to send data to slave
  * retval none
  */

void ENC_SPI_SendBuf(uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize)
{
    /* Select ENC28J60 module */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_RESET);
    up_udelay(1);

    /* Transmit or receuve data */
    if (slave2master == NULL) {
        if (master2slave != NULL) {
            HAL_SPI_Transmit(&hspi2, master2slave, bufferSize, 1000);
        }
    } else if (master2slave == NULL) {
        HAL_SPI_Receive(&hspi2, slave2master, bufferSize, 1000);
    } else {
        HAL_SPI_TransmitReceive(&hspi2, master2slave, slave2master, bufferSize, 1000);
    }

    /* De-select ENC28J60 module */
    HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_SET);
    up_udelay(1);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * Implement SPI Slave selection and deselection. Must be provided by user code
  * param  select: true if the ENC28J60 slave SPI if selected, false otherwise
  * retval none
  */

void ENC_SPI_Select(bool select)
{
    /* Select or de-select ENC28J60 module */
    if (select) {
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_RESET);
        up_udelay(1);
    } else {
        HAL_GPIO_WritePin(GPIOB, ENC_CS_PIN, GPIO_PIN_SET);
        up_udelay(1);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
