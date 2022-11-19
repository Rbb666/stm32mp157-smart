/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-07-20     thread-liu        the first version
 */

#include "board.h"
#include "drv_config.h"
#include <netif/ethernetif.h>
#include "lwipopts.h"
#include "drv_eth.h"

#if defined(BSP_USING_GBE)

#define DRV_DEBUG
#define ETH_RX_DUMP
#define ETH_TX_DUMP
#define LOG_TAG             "drv.emac"
#include <drv_log.h>

#define MAX_ADDR_LEN 6
rt_base_t level;

#define TX_ADD_BASE         0x2FFC3000
#define RX_ADD_BASE         0x2FFC5000
#define TX_DMA_ADD_BASE     0x2FFC7000
#define RX_DMA_ADD_BASE     0x2FFC7100

#if defined ( __GNUC__ )
/* transmit buffer */
static rt_uint8_t txBuffer[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((at(TX_ADD_BASE)));
/* Receive buffer */
static rt_uint8_t rxBuffer[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((at(RX_ADD_BASE)));
/* Transmit DMA descriptors */
static TxDmaDesc txDmaDesc[ETH_TXBUFNB] __attribute__((at(TX_DMA_ADD_BASE)));
/* Receive DMA descriptors */
static RxDmaDesc rxDmaDesc[ETH_RXBUFNB] __attribute__((at(RX_DMA_ADD_BASE)));
#endif

//Current transmit descriptor
static rt_uint8_t txIndex = 0;
//Current receive descriptor
static rt_uint8_t rxIndex = 0;

/* eth */
static struct rt_event rx_event = {0};

#define ETH_TIME_OUT 100000

struct rt_stm32_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;
#ifndef PHY_USING_INTERRUPT_MODE
    rt_timer_t poll_link_timer;
#endif
    /* interface address info, hw address */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];
    /* eth speed */
    uint32_t    eth_speed;
    /* eth duplex mode */
    uint32_t    eth_mode;

    void *descs;
    volatile struct eqos_desc *tx_descs;
    volatile struct eqos_desc *rx_descs;

    void *descs_phy;

    struct eqos_desc *tx_descs_phy;
    struct eqos_desc *rx_descs_phy;

    void *tx_dma_buf;
    void *rx_dma_buf;

    void *tx_dma_buf_phy;
    void *rx_dma_buf_phy;
};
static struct rt_stm32_eth stm32_eth_device = {0};

rt_uint32_t uid_addr;
SYSCFG_TypeDef *syscfg_addr;
ETH_TypeDef *eth_addr;

void ETH1_IRQHandler(int irqno, void *param);

void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    LOG_E("Error_Handler at file:%s num:%d", s, num);
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

#if defined(ETH_RX_DUMP) || defined(ETH_TX_DUMP)
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static void dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;

    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}
#endif

static rt_err_t phy_write_reg(uint8_t phy_addr, uint8_t reg_addr, uint16_t reg_value)
{
    uint32_t temp;
    volatile uint32_t tickstart = 0;
    /* Take care not to alter MDC clock configuration */
    temp = eth_addr->MACMDIOAR & ETH_MACMDIOAR_CR;
    /* Set up a write operation */
    temp |= ETH_MACMDIOAR_GOC_Val(1) | ETH_MACMDIOAR_GB;
    /* PHY address */
    temp |= (phy_addr << 21) & ETH_MACMDIOAR_PA;
    /* Register address */
    temp |= (reg_addr << 16) & ETH_MACMDIOAR_RDA;

    /* Data to be written in the PHY register */
    eth_addr->MACMDIODR = reg_value & ETH_MACMDIODR_GD;

    /* Start a write operation */
    eth_addr->MACMDIOAR = temp;
    /* Wait for the write to complete */
    tickstart = rt_tick_get();
    while((eth_addr->MACMDIOAR & ETH_MACMDIOAR_GB) != 0)
    {
        /* judge timeout */
        if((rt_tick_get() - tickstart) > ETH_TIME_OUT)
        {
            LOG_E("PHY write reg %02x date %04x timeout!", reg_addr, reg_value);
            return RT_ETIMEOUT;
        }
    }

    return RT_EOK;
}

static uint16_t phy_read_reg(uint8_t phy_addr, uint8_t reg_addr)
{
    uint16_t reg_value = 0;
    uint32_t status = 0;
    volatile uint32_t tickstart = 0;

    /* Take care not to alter MDC clock configuration */
    status = eth_addr->MACMDIOAR & ETH_MACMDIOAR_CR;
    /* Set up a read operation */
    status |= ETH_MACMDIOAR_GOC_Val(3) | ETH_MACMDIOAR_GB;
    /* PHY address */
    status |= (phy_addr << 21) & ETH_MACMDIOAR_PA;
    /* Register address */
    status |= (reg_addr << 16) & ETH_MACMDIOAR_RDA;

    /* Start a read operation */
    eth_addr->MACMDIOAR = status;
    /* Wait for the read to complete */
    tickstart = rt_tick_get();
    while((eth_addr->MACMDIOAR & ETH_MACMDIOAR_GB) != 0)
    {
        /* judge timeout */
        if((rt_tick_get() - tickstart) > ETH_TIME_OUT)
        {
            LOG_E("PHY read reg %02x timeout!", reg_addr);
            return RT_ETIMEOUT;
        }
    }

    /* Get register value */
    reg_value = eth_addr->MACMDIODR & ETH_MACMDIODR_GD;

    return reg_value;
}

static rt_err_t update_mac_mode(void)
{
   uint32_t status;

   /* Read current MAC configuration */
   status = eth_addr->MACCR;

   if (stm32_eth_device.eth_speed & PHY_1000M)
   {
      status &= ~ETH_MACCR_PS;
      status &= ~ETH_MACCR_FES;
   }
   else if (stm32_eth_device.eth_speed & PHY_100M)
   {
      status |= ETH_MACCR_PS;
      status |= ETH_MACCR_FES;
   }
   /* 10M */
   else
   {
      status |= ETH_MACCR_PS;
      status &= ~ETH_MACCR_FES;
   }

   if (stm32_eth_device.eth_mode & PHY_FULL_DUPLEX)
   {
      status |= ETH_MACCR_DM;
   }
   else
   {
      status &= ~ETH_MACCR_DM;
   }

   /* Update MAC configuration register */
   eth_addr->MACCR = status;

   return RT_EOK;
}

#define ST_IS_ENGINEERING_BOOT_MODE()   (((syscfg_addr->BOOTR) & (SYSCFG_BOOTR_BOOT2|SYSCFG_BOOTR_BOOT1|SYSCFG_BOOTR_BOOT0)) == (SYSCFG_BOOTR_BOOT2))

static void HAL_ETH_MspInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if(ST_IS_ENGINEERING_BOOT_MODE())
    {
        rt_kprintf("warning:IS_ENGINEERING_BOOT_MODE!!");
        /** Initializes the peripherals clock
        */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ETH;
        PeriphClkInit.EthClockSelection = RCC_ETHCLKSOURCE_PLL4;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* Enable SYSCFG clock */
    rcc_addr->MP_APB3ENSETR = RCC_MP_APB3ENSETR_SYSCFGEN;

    /* Select RGMII interface mode */
    // HAL_SYSCFG_ETHInterfaceSelect(SYSCFG_ETH_RGMII);
    assert_param(IS_SYSCFG_ETHERNET_CONFIG(SYSCFG_ETH_RGMII));
    syscfg_addr->PMCCLRR = SYSCFG_PMCSETR_ETH_SEL|SYSCFG_PMCSETR_ETH_SELMII_SEL;
    syscfg_addr->PMCSETR = (uint32_t)(SYSCFG_ETH_RGMII);

    /* Enable Ethernet MAC clock */
    rcc_addr->MP_AHB6ENSETR = RCC_MP_AHB6ENSETR_ETHMACEN;
    rcc_addr->MP_AHB6ENSETR = RCC_MP_AHB6ENSETR_ETHTXEN;
    rcc_addr->MP_AHB6ENSETR = RCC_MP_AHB6ENSETR_ETHRXEN;

    /**ETH1 GPIO Configuration
    PA1     ------>  ETH1_RX_CLK
    PA2     ------>  ETH1_MDIO
    PA7     ------>  ETH1_RX_CTL
    PB0     ------>  ETH1_RXD2
    PB1     ------>  ETH1_RXD3
    PB11    ------>  ETH1_TX_CTL
    PC1     ------>  ETH1_MDC
    PC2     ------>  ETH1_TXD2
    PC4     ------>  ETH1_RXD0
    PC5     ------>  ETH1_RXD1
    PE2     ------>  ETH1_TXD3
    PG4     ------>  ETH1_GTX_CLK
    PG5     ------>  ETH1_CLK125
    PG13    ------>  ETH1_TXD0
    PG14    ------>  ETH1_TXD1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(mask_tab[0].gpio_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11;
    HAL_GPIO_Init(mask_tab[1].gpio_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
    HAL_GPIO_Init(mask_tab[2].gpio_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(mask_tab[4].gpio_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_14;
    HAL_GPIO_Init(mask_tab[6].gpio_port, &GPIO_InitStruct);

    /* Configure PHY_RST (PI10) */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(mask_tab[8].gpio_port, &GPIO_InitStruct);

    /* Reset PHY transceiver */
    HAL_GPIO_WritePin(mask_tab[8].gpio_port, GPIO_PIN_10, GPIO_PIN_RESET);
    rt_thread_mdelay(50);
    HAL_GPIO_WritePin(mask_tab[8].gpio_port, GPIO_PIN_10, GPIO_PIN_SET);
    rt_thread_mdelay(40);
}

static int enet_buffer_init(struct rt_stm32_eth *eqos)
{
    void *tx_rx_descs = RT_NULL;
    void *tx_rx_descs_nocache = RT_NULL; 

    //
    tx_rx_descs = rt_pages_alloc(RX_BD_INDEX_NUM);
    void *tx_rx_descs_phy = RT_NULL;
    if(!tx_rx_descs)
    {
        LOG_E("ERROR: rx buff page alloc failed\n");
        return RT_ERROR;
    }
    tx_rx_descs_nocache = (void *)rt_ioremap_nocache(virtual_to_physical(tx_rx_descs), (SYS_PAGE_SIZE<<RX_BD_INDEX_NUM));
    tx_rx_descs_phy = (void *)virtual_to_physical(tx_rx_descs);

    //
    void *tx_dma_buffer = RT_NULL;
    void *tx_dma_buffer_phy = RT_NULL;

    tx_dma_buffer = rt_pages_alloc(TX_BUFFER_INDEX_NUM);
    if(!tx_dma_buffer)
    {
        LOG_E("ERROR: rx buff page alloc failed\n");
        return RT_ERROR;
    }
    tx_dma_buffer_phy = (void *)virtual_to_physical(tx_dma_buffer);

    //
    void *rx_dma_buffer = RT_NULL;
    void *rx_dma_buffer_phy = RT_NULL;

    rx_dma_buffer = rt_pages_alloc(RX_BUFFER_INDEX_NUM);
    if(!rx_dma_buffer)
    {
        LOG_E("ERROR: rx buff page alloc failed\n");
        return RT_ERROR;
    }
    rx_dma_buffer_phy = (void *)virtual_to_physical(rx_dma_buffer);

    eqos->descs = (void *)tx_rx_descs_nocache;
    eqos->descs_phy = tx_rx_descs_phy;

    eqos->tx_descs = (struct eqos_desc *)eqos->descs;    
    eqos->rx_descs = (eqos->tx_descs + EQOS_DESCRIPTORS_TX);

    eqos->tx_descs_phy = (struct eqos_desc *)eqos->descs_phy;    
    eqos->rx_descs_phy = (eqos->tx_descs_phy + EQOS_DESCRIPTORS_TX);


    eqos->tx_dma_buf = (void *)tx_dma_buffer;
    eqos->rx_dma_buf = (void *)rx_dma_buffer;

    eqos->tx_dma_buf_phy = (void *)tx_dma_buffer_phy;
    eqos->rx_dma_buf_phy = (void *)rx_dma_buffer_phy;
}

static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);

    rt_uint32_t status;
    int i = 0 ;
    volatile uint32_t tickstart = 0;
    uint8_t  *macAddr = &stm32_eth_device.dev_addr[0];

    /* Initialize RX/TX descriptor index */
    rxIndex = txIndex = 0;

    HAL_ETH_MspInit();

    /* Reset Ethernet MAC peripheral */
    rcc_addr->AHB6RSTSETR = RCC_AHB6RSTSETR_ETHMACRST;
    rcc_addr->AHB6RSTCLRR = RCC_AHB6RSTCLRR_ETHMACRST;

    /* Ethernet Software reset */
    eth_addr->DMAMR |= ETH_DMAMR_SWR;

    /* Wait for the reset to complete */
    tickstart = rt_tick_get();
    while (READ_BIT(eth_addr->DMAMR, ETH_DMAMR_SWR))
    {
        if(((HAL_GetTick() - tickstart ) > ETH_TIME_OUT))
        {
            LOG_E("ETH software reset timeout!");
            return RT_ERROR;
        }
    }
    /* Adjust MDC clock range depending on HCLK frequency */
    eth_addr->MACMDIOAR = ETH_MACMDIOAR_CR_Val(5);

    /* Use default MAC configuration */
    eth_addr->MACCR = ETH_MACCR_DO;

    /* Set the MAC address of the station */
    eth_addr->MACA0LR = ((macAddr[3] << 24) | (macAddr[2] << 16) | (macAddr[1] << 8) | macAddr[0]);
    eth_addr->MACA0HR = ((macAddr[5] << 8) | macAddr[4]);

    /* The MAC supports 3 additional addresses for unicast perfect filtering */
    eth_addr->MACA1LR = 0;
    eth_addr->MACA1HR = 0;
    eth_addr->MACA2LR = 0;
    eth_addr->MACA2HR = 0;
    eth_addr->MACA3LR = 0;
    eth_addr->MACA3HR = 0;

    /* Initialize hash table */
    eth_addr->MACHT0R = 0;
    eth_addr->MACHT1R = 0;

    /* Configure the receive filter */
    eth_addr->MACPFR = ETH_MACPFR_HPF | ETH_MACPFR_HMC;

    /* Disable flow control */
    eth_addr->MACQ0TXFCR = 0;
    eth_addr->MACRXFCR   = 0;

    /* Enable the first RX queue */
    eth_addr->MACRXQC0R = ETH_MACRXQC0R_RXQ0EN_Val(1);

    /* Configure DMA operating mode */
    eth_addr->DMAMR = ETH_DMAMR_INTM_Val(0) | ETH_DMAMR_PR_Val(0);

    /* Configure system bus mode */
    eth_addr->DMASBMR |= ETH_DMASBMR_AAL;

    /* The DMA takes the descriptor table as contiguous */
    eth_addr->DMAC0CR = ETH_DMAC0CR_DSL_Val(0);

    /* Configure TX features */
    eth_addr->DMAC0TXCR = ETH_DMAC0TXCR_TXPBL_Val(1);

    /* Configure RX features */
    eth_addr->DMAC0RXCR = ETH_DMAC0RXCR_RXPBL_Val(1) | ETH_DMAC0RXCR_RBSZ_Val(ETH_RX_BUF_SIZE);

    /* Enable store and forward mode for transmission */
    eth_addr->MTLTXQ0OMR = ETH_MTLTXQ0OMR_TQS_Val(7) | ETH_MTLTXQ0OMR_TXQEN_Val(2) | ETH_MTLTXQ0OMR_TSF;

    /* Enable store and forward mode for reception */
    eth_addr->MTLRXQ0OMR = ETH_MTLRXQ0OMR_RQS_Val(7) | ETH_MTLRXQ0OMR_RSF;

    /* Initialize TX DMA descriptor list */
    for (i = 0; i < ETH_TXBUFNB; i++)
    {
        /* The descriptor is initially owned by the application */
        txDmaDesc[i].tdes0 = 0;
        txDmaDesc[i].tdes1 = 0;
        txDmaDesc[i].tdes2 = 0;
        txDmaDesc[i].tdes3 = 0;
    }

    /* Initialize RX DMA descriptor list */
    for (i = 0; i < ETH_RXBUFNB; i++)
    {
        /* The descriptor is initially owned by the DMA */
        rxDmaDesc[i].rdes0 = (uint32_t)rxBuffer[i];
        rxDmaDesc[i].rdes1 = 0;
        rxDmaDesc[i].rdes2 = 0;
        rxDmaDesc[i].rdes3 = ETH_RDES3_OWN | ETH_RDES3_IOC | ETH_RDES3_BUF1V;
    }

    /* Set Transmit Descriptor List Address Register */
    eth_addr->DMAC0TXDLAR = (uint32_t)&txDmaDesc[0];
    /* Length of the transmit descriptor ring */
    eth_addr->DMAC0TXRLR = ETH_TXBUFNB - 1;

    /* Set Receive Descriptor List Address Register */
    eth_addr->DMAC0RXDLAR = (uint32_t)&rxDmaDesc[0];
    /* Length of the receive descriptor ring */
    eth_addr->DMAC0RXRLR = ETH_RXBUFNB - 1;

    /* Prevent interrupts from being generated when the transmit statistic
     * counters reach half their maximum value */
    eth_addr->MMCTXIMR = ETH_MMCTXIMR_TXLPITRCIM | ETH_MMCTXIMR_TXLPIUSCIM | ETH_MMCTXIMR_TXGPKTIM | ETH_MMCTXIMR_TXMCOLGPIM | ETH_MMCTXIMR_TXSCOLGPIM;

    /* Prevent interrupts from being generated when the receive statistic
     * counters reach half their maximum value */
    eth_addr->MMCRXIMR = ETH_MMCRXIMR_RXLPITRCIM | ETH_MMCRXIMR_RXLPIUSCIM | ETH_MMCRXIMR_RXUCGPIM | ETH_MMCRXIMR_RXALGNERPIM | ETH_MMCRXIMR_RXCRCERPIM;

    /* Disable MAC interrupts */
    eth_addr->MACIER = 0;

    /* Enable the desired DMA interrupts */
    eth_addr->DMAC0IER = ETH_DMAC0IER_NIE | ETH_DMAC0IER_RIE | ETH_DMAC0IER_TIE;

    /* Enable MAC transmission and reception */
    eth_addr->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;

    /* Enable DMA transmission and reception */
    eth_addr->DMAC0TXCR |= ETH_DMAC0TXCR_ST;
    eth_addr->DMAC0RXCR |= ETH_DMAC0RXCR_SR;

    /* Reset PHY transceiver */
    phy_write_reg(RTL8211F_PHY_ADDR, RTL8211F_BMCR, RTL8211F_BMCR_RESET);
    status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_BMCR);
    /* Wait for the reset to complete */
    tickstart = rt_tick_get();
    while (status & RTL8211F_BMCR_RESET)
    {
        if((rt_tick_get() - tickstart) > ETH_TIME_OUT)
        {
            LOG_E("PHY software reset timeout!");
            return RT_ETIMEOUT;
        }
        else
        {
            status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_BMCR);
        }
    }

    LOG_I("phy reg:%d", status);

    /* The PHY will generate interrupts when link status changes are detected */
    phy_write_reg(RTL8211F_PHY_ADDR, RTL8211F_INER, RTL8211F_INER_AN_COMPLETE | RTL8211F_INER_LINK_STATUS);

    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_D("emac open");
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
    LOG_D("emac close");
    return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("emac read");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_stm32_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("emac write");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args)
        {
            rt_memcpy(args, stm32_eth_device.dev_addr, 6);
        }
        else
        {
            return -RT_ERROR;
        }
        break;

    default :
        break;
    }

    return RT_EOK;
}

rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p)
{
    uint32_t framelen = 0;
    struct pbuf *q = RT_NULL;
    struct rt_stm32_eth *eqos = &stm32_eth_device;

    /* Copy user data to the transmit buffer */
    for (q = p; q != NULL; q = q->next)
    {
        /* Make sure the current buffer is available for writing */
        if((txDmaDesc[txIndex].tdes3 & ETH_TDES3_OWN) != 0)
        {
            LOG_D("buffer not valid");
            return ERR_USE;
        }

        level = rt_hw_interrupt_disable();
        rt_memcpy(&txBuffer[txIndex][framelen], q->payload, q->len);
        framelen += q->len;
        rt_hw_interrupt_enable(level);

        /* Check the frame length */
        if (framelen > ETH_TX_BUF_SIZE - 1)
        {
            LOG_D(" tx buffer frame length over : %d", framelen);
            return ERR_USE;
        }
    }

#ifdef ETH_TX_DUMP
        rt_kprintf("Tx dump, len= %d\r\n", framelen);
        dump_hex(txBuffer[txIndex], framelen);
#endif

    /* Set the start address of the buffer */
    txDmaDesc[txIndex].tdes0 = (uint32_t)txBuffer[txIndex];
    /* Write the number of bytes to send */
    txDmaDesc[txIndex].tdes2 = ETH_TDES2_IOC | (framelen & ETH_TDES2_B1L);
    /* Give the ownership of the descriptor to the DMA */
    txDmaDesc[txIndex].tdes3 = ETH_TDES3_OWN | ETH_TDES3_FD | ETH_TDES3_LD;

    /* Data synchronization barrier */
    __DSB();

    /* Clear TBU flag to resume processing */
    eth_addr->DMAC0SR = ETH_DMAC0SR_TBU;
    /* Instruct the DMA to poll the transmit descriptor list */
    eth_addr->DMAC0TXDTPR = 0;

    if (++txIndex > ETH_TXBUFNB - 1)
    {
        txIndex = 0;
    }

    return ERR_OK;
}

struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
    rt_uint32_t framelength = 0;
    uint32_t framelen = 0;
    struct pbuf *p = RT_NULL, *q = RT_NULL;

    LOG_I("rxIndex:%d,rxDmaDesc rdes3:%#x", rxIndex, rxDmaDesc[rxIndex].rdes3);
    /* The current buffer is available for reading */
    if (!(rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_OWN))
    {
        /* FD and LD flags should be set */
        if ((rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_FD) && (rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_LD))
            {
                /* Make sure no error occurred */
                if(!(rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_ES))
                {
                    /* Retrieve the length of the frame */
                    framelength = rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_PL;
                    /* check the frame length */
                    framelength = (framelength > ETH_RX_BUF_SIZE) ? ETH_RX_BUF_SIZE : framelength;
                    p = pbuf_alloc(PBUF_RAW, framelength, PBUF_RAM);
                    if (p != NULL)
                    {
                        for (q = p; q != NULL; q = q->next)
                        {
                            level=rt_hw_interrupt_disable();
                            rt_memcpy(q->payload, &rxBuffer[rxIndex][framelen], q->len);
                            framelen += q->len;
                            rt_hw_interrupt_enable(level);

                            if (framelen > framelength)
                            {
                                LOG_E("frame len is too long!");
                                return RT_NULL;
                            }
                        }
                    }
                }
                else
                {
                    /* The received packet contains an error */
                    LOG_D("the received packet contains an error!");
                    return RT_NULL;
                }

            }
        else
        {
            /* The packet is not valid */
            LOG_D("the packet is not valid");
            return RT_NULL;
        }

        /* Set the start address of the buffer */
        rxDmaDesc[rxIndex].rdes0 = (uint32_t)rxBuffer[rxIndex];
        /* Give the ownership of the descriptor back to the DMA */
        rxDmaDesc[rxIndex].rdes3 = ETH_RDES3_OWN | ETH_RDES3_IOC | ETH_RDES3_BUF1V;
#ifdef ETH_RX_DUMP
        rt_kprintf("Rx dump, len= %d\r\n", framelen);
        dump_hex(rxBuffer[rxIndex], framelen);
#endif
        /* Increment index and wrap around if necessary */
        if (++rxIndex > ETH_RXBUFNB - 1)
        {
            rxIndex = 0;
        }
        /* Clear RBU flag to resume processing */
        eth_addr->DMAC0SR = ETH_DMAC0SR_RBU;
        /* Instruct the DMA to poll the receive descriptor list */
        eth_addr->DMAC0RXDTPR = 0;
    }

    return p;
}

void ETH1_IRQHandler(int irqno, void *param)
{
    rt_uint32_t status = 0;

    LOG_I("enter eth1 irqhander!!!!!!!!!!!!\n");

    /* enter interrupt */
    rt_interrupt_enter();
    /* Read DMA status register */
    status = eth_addr->DMAC0SR;
    /* Frame transmitted */
    if (status & ETH_DMAC0SR_TI)
    {
        /* Clear the Eth DMA Tx IT pending bits */
        eth_addr->DMAC0SR = ETH_DMAC0SR_TI;
    }
    /* Frame received */
    else if (status & ETH_DMAC0SR_RI)
    {
        /* Disable RIE interrupt */
        eth_addr->DMAC0IER &= ~ETH_DMAC0IER_RIE;

        rt_event_send(&rx_event, status);
    }
    /* eth_addr DMA Error */
    if (status & ETH_DMAC0SR_AIS)
    {
        eth_addr->DMAC0IER &= ~ETH_DMAC0IER_AIE;
        LOG_E("eth dam err");
    }
    /* Clear the interrupt flags */
    eth_addr->DMAC0SR = ETH_DMAC0SR_NIS;

    /* leave interrupt */
    rt_interrupt_leave();
}

static void phy_linkchange()
{
    rt_uint32_t status = 0;

    /* Read status register to acknowledge the interrupt */
    status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_INSR);

    if (status & (RTL8211F_INSR_AN_COMPLETE | RTL8211F_INSR_LINK_STATUS))
    {
        status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_BMSR);
        status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_BMSR);

        if (status & RTL8211F_BMSR_LINK_STATUS)
        {
            LOG_D("link up");

            status = phy_read_reg(RTL8211F_PHY_ADDR, RTL8211F_PHYSR);

            switch (status & RTL8211F_PHYSR_SPEED)
            {
            case RTL8211F_PHYSR_SPEED_10MBPS:
            {
                LOG_D("speed: 10M");
                stm32_eth_device.eth_speed |= PHY_10M;
            break;
            }

            case RTL8211F_PHYSR_SPEED_100MBPS:
            {
                LOG_D("speed: 100M");
                stm32_eth_device.eth_speed |= PHY_100M;
            break;
            }

            case RTL8211F_PHYSR_SPEED_1000MBPS:
            {
                LOG_D("speed: 1000M");
                stm32_eth_device.eth_speed |= PHY_1000M;
            break;
            }

            /* Unknown speed */
            default:
                rt_kprintf("Invalid speed.");
            break;
            }

            stm32_eth_device.eth_mode  = (status & RTL8211F_PHYSR_DUPLEX)? PHY_FULL_DUPLEX : PHY_HALF_DUPLEX;
            update_mac_mode();
            /* send link up. */
            eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
        }
        else
        {
            LOG_D("link down");
            eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
        }
    }
}

#ifdef PHY_USING_INTERRUPT_MODE
static void eth_phy_isr(void *args)
{
    rt_uint32_t status = 0;

    phy_read_reg(RTL8211F_PHY_ADDR, PHY_INTERRUPT_FLAG_REG, (uint32_t *)&status);
    LOG_D("phy interrupt status reg is 0x%X", status);

    phy_linkchange();
}
#endif /* PHY_USING_INTERRUPT_MODE */

static void phy_monitor_thread_entry(void *parameter)
{
    rt_uint32_t status = 0;

    phy_linkchange();
#ifdef PHY_USING_INTERRUPT_MODE
    /* configuration intterrupt pin */
    rt_pin_mode(PHY_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PHY_INT_PIN, PIN_IRQ_MODE_FALLING, eth_phy_isr, (void *)"callbackargs");
    rt_pin_irq_enable(PHY_INT_PIN, PIN_IRQ_ENABLE);

    /* enable phy interrupt */
    phy_write_reg(RTL8211F_PHY_ADDR, PHY_INTERRUPT_MASK_REG, PHY_INT_MASK);
#if defined(PHY_INTERRUPT_CTRL_REG)
    phy_write_reg( RTL8211F_PHY_ADDR, PHY_INTERRUPT_CTRL_REG, PHY_INTERRUPT_EN);
#endif
#else /* PHY_USING_INTERRUPT_MODE */
    stm32_eth_device.poll_link_timer = rt_timer_create("phylnk", (void (*)(void*))phy_linkchange,
                                        NULL, RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
    if (!stm32_eth_device.poll_link_timer || rt_timer_start(stm32_eth_device.poll_link_timer) != RT_EOK)
    {
        LOG_E("Start link change detection timer failed");
    }
#endif /* PHY_USING_INTERRUPT_MODE */
    while(1)
    {
        if (rt_event_recv(&rx_event, 0xffffffff, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &status) == RT_EOK)
        {
            /* check dma rx buffer */
            if (eth_addr->DMAC0SR & ETH_DMAC0SR_RI)
            {
                /* Clear interrupt flag */
                eth_addr->DMAC0SR = ETH_DMAC0SR_RI;
                /* Process all pending packets */
                while (rxDmaDesc[rxIndex].rdes3 & ETH_RDES3_PL)
                {
                    /* trigger lwip receive thread */
                    eth_device_ready(&(stm32_eth_device.parent));
                }
            }

            /* enable DMA interrupts */
            eth_addr->DMAC0IER = ETH_DMAC0IER_NIE | ETH_DMAC0IER_RIE | ETH_DMAC0IER_TIE;
        }
    }
}

static const struct rt_device_ops stm32_eth_ops =
{
    rt_stm32_eth_init,
    rt_stm32_eth_open,
    rt_stm32_eth_close,
    rt_stm32_eth_read,
    rt_stm32_eth_write,
    rt_stm32_eth_control,
};

/* Register the EMAC device */
static int rt_hw_stm32_eth_init(void)
{
    rt_err_t state = RT_EOK;

    uid_addr = (rt_uint32_t)rt_ioremap((void *)UID_BASE, 0x1000);
    syscfg_addr = (SYSCFG_TypeDef *)rt_ioremap((void *)SYSCFG, 0x400);
    eth_addr = (ETH_TypeDef *)rt_ioremap((void *)ETH, 0x2000);

    /* ETH interrupt Init */
    rt_hw_interrupt_install(ETH1_IRQn, ETH1_IRQHandler, RT_NULL, "ETH1_IRQ");
    rt_hw_interrupt_umask(ETH1_IRQn);

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0x00;
    stm32_eth_device.dev_addr[1] = 0x80;
    stm32_eth_device.dev_addr[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID. */
    stm32_eth_device.dev_addr[3] = *(rt_uint8_t *)(uid_addr + 4);
    stm32_eth_device.dev_addr[4] = *(rt_uint8_t *)(uid_addr + 2);
    stm32_eth_device.dev_addr[5] = *(rt_uint8_t *)(uid_addr + 0);

    stm32_eth_device.parent.parent.ops       = &stm32_eth_ops;
    stm32_eth_device.parent.parent.user_data = RT_NULL;

    stm32_eth_device.parent.eth_rx = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx = rt_stm32_eth_tx;

    rt_event_init(&rx_event, "eth_rx", RT_IPC_FLAG_FIFO);

    /* register eth device */
    state = eth_device_init(&(stm32_eth_device.parent), "e0");
    if (RT_EOK == state)
    {
        LOG_D("emac device init success");
    }
    else
    {
        LOG_E("emac device init faild: %d", state);
        state = -RT_ERROR;
    }

    /* start phy monitor */
    rt_thread_t tid;
    tid = rt_thread_create("phy",
                           phy_monitor_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           2);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        state = -RT_ERROR;
    }

    return state;
}

INIT_DEVICE_EXPORT(rt_hw_stm32_eth_init);

#endif
