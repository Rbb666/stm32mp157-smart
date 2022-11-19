/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-07-20     thread-liu        the first version
 */

#ifndef __DRV_ETH_H__
#define __DRV_ETH_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Transmit descriptor
 **/
typedef struct
{
    uint32_t tdes0;
    uint32_t tdes1;
    uint32_t tdes2;
    uint32_t tdes3;
} TxDmaDesc;

/**
 * @brief Receive descriptor
 **/
typedef struct
{
    uint32_t rdes0;
    uint32_t rdes1;
    uint32_t rdes2;
    uint32_t rdes3;
} RxDmaDesc;

enum {
    PHY_LINK        = (1 << 0),
    PHY_10M         = (1 << 1),
    PHY_100M        = (1 << 2),
    PHY_1000M       = (1 << 3),
    PHY_FULL_DUPLEX = (1 << 4),
    PHY_HALF_DUPLEX = (1 << 5)
};

struct eqos_desc {
    volatile uint32_t des0;
    volatile uint32_t des1;
    volatile uint32_t des2;
    volatile uint32_t des3;
};

#define EQOS_DESCRIPTORS_TX    32
#define EQOS_DESCRIPTORS_RX    32

#define virtual_to_physical(v) ((void *)((size_t)v + PV_OFFSET))
#define physical_to_virtual(p) ((void *)((size_t)p - PV_OFFSET))
#define SYS_PAGE_SIZE (4096)

#define TX_BUFFER_INDEX_NUM (6)
#define RX_BUFFER_INDEX_NUM (6)
#define TX_BD_INDEX_NUM (2)
#define RX_BD_INDEX_NUM (2)

#define RTL8211F_PHY_ADDR       1           /* PHY address */

#define ETH_TXBUFNB             4           /* 4 Tx buffers of size ETH_TX_BUF_SIZE */
#define ETH_TX_BUF_SIZE         1536        /* buffer size for transmit */
#define ETH_RXBUFNB             4           /* 4 Rx buffers of size ETH_RX_BUF_SIZE */
#define ETH_RX_BUF_SIZE         1536        /* buffer size for receive */

#define ETH_MMC_INTERRUPT_MASK_TXLPITRCIM_Msk ETH_MMCTXIMR_TXLPITRCIM_Msk /* ETH_MMCTXIMR register  */

/* Register access macros */
#define ETH_MACRXQC0R_RXQ0EN_Val(n)     (((n) << ETH_MACRXQC0R_RXQ0EN_Pos) & ETH_MACRXQC0R_RXQ0EN_Msk)
#define ETH_MACMDIOAR_CR_Val(n)         (((n) << ETH_MACMDIOAR_CR_Pos) & ETH_MACMDIOAR_CR_Msk)
#define ETH_MACMDIOAR_GOC_Val(n)        (((n) << ETH_MACMDIOAR_GOC_Pos) & ETH_MACMDIOAR_GOC_Msk)
#define ETH_MTLTXQ0OMR_TQS_Val(n)       (((n) << ETH_MTLTXQ0OMR_TQS_Pos) & ETH_MTLTXQ0OMR_TQS_Msk)
#define ETH_MTLTXQ0OMR_TXQEN_Val(n)     (((n) << ETH_MTLTXQ0OMR_TXQEN_Pos) & ETH_MTLTXQ0OMR_TXQEN_Msk)
#define ETH_MTLRXQ0OMR_RQS_Val(n)       (((n) << ETH_MTLRXQ0OMR_RQS_Pos) & ETH_MTLRXQ0OMR_RQS_Msk)
#define ETH_DMAMR_INTM_Val(n)           (((n) << ETH_DMAMR_INTM_Pos) & ETH_DMAMR_INTM_Msk)
#define ETH_DMAMR_PR_Val(n)             (((n) << ETH_DMAMR_PR_Pos) & ETH_DMAMR_PR_Msk)
#define ETH_DMAC0CR_DSL_Val(n)          (((n) << ETH_DMAC0CR_DSL_Pos) & ETH_DMAC0CR_DSL_Msk)
#define ETH_DMAC0TXCR_TXPBL_Val(n)      (((n) << ETH_DMAC0TXCR_TXPBL_Pos) & ETH_DMAC0TXCR_TXPBL_Msk)
#define ETH_DMAC0RXCR_RXPBL_Val(n)      (((n) << ETH_DMAC0RXCR_RXPBL_Pos) & ETH_DMAC0RXCR_RXPBL_Msk)
#define ETH_DMAC0RXCR_RBSZ_Val(n)       (((n) << ETH_DMAC0RXCR_RBSZ_Pos) & ETH_DMAC0RXCR_RBSZ_Msk)

/* Transmit normal descriptor (read format) */
#define ETH_TDES0_BUF1AP        0xFFFFFFFF
#define ETH_TDES1_BUF2AP        0xFFFFFFFF
#define ETH_TDES2_IOC           0x80000000
#define ETH_TDES2_TTSE          0x40000000
#define ETH_TDES2_B2L           0x3FFF0000
#define ETH_TDES2_VTIR          0x0000C000
#define ETH_TDES2_B1L           0x00003FFF
#define ETH_TDES3_OWN           0x80000000
#define ETH_TDES3_CTXT          0x40000000
#define ETH_TDES3_FD            0x20000000
#define ETH_TDES3_LD            0x10000000
#define ETH_TDES3_CPC           0x0C000000
#define ETH_TDES3_SAIC          0x03800000
#define ETH_TDES3_THL           0x00780000
#define ETH_TDES3_TSE           0x00040000
#define ETH_TDES3_CIC           0x00030000
#define ETH_TDES3_FL            0x00007FFF

/* Transmit normal descriptor (write-back format) */
#define ETH_TDES0_TTSL          0xFFFFFFFF
#define ETH_TDES1_TTSH          0xFFFFFFFF
#define ETH_TDES3_OWN           0x80000000
#define ETH_TDES3_CTXT          0x40000000
#define ETH_TDES3_FD            0x20000000
#define ETH_TDES3_LD            0x10000000
#define ETH_TDES3_TTSS          0x00020000
#define ETH_TDES3_ES            0x00008000
#define ETH_TDES3_JT            0x00004000
#define ETH_TDES3_FF            0x00002000
#define ETH_TDES3_PCE           0x00001000
#define ETH_TDES3_LOC           0x00000800
#define ETH_TDES3_NC            0x00000400
#define ETH_TDES3_LC            0x00000200
#define ETH_TDES3_EC            0x00000100
#define ETH_TDES3_CC            0x000000F0
#define ETH_TDES3_ED            0x00000008
#define ETH_TDES3_UF            0x00000004
#define ETH_TDES3_DB            0x00000002
#define ETH_TDES3_IHE           0x00000001

/* Transmit context descriptor           */
#define ETH_TDES0_TTSL          0xFFFFFFFF
#define ETH_TDES1_TTSH          0xFFFFFFFF
#define ETH_TDES2_IVT           0xFFFF0000
#define ETH_TDES2_MSS           0x00003FFF
#define ETH_TDES3_OWN           0x80000000
#define ETH_TDES3_CTXT          0x40000000
#define ETH_TDES3_OSTC          0x08000000
#define ETH_TDES3_TCMSSV        0x04000000
#define ETH_TDES3_CDE           0x00800000
#define ETH_TDES3_IVLTV         0x00020000
#define ETH_TDES3_VLTV          0x00010000
#define ETH_TDES3_VT            0x0000FFFF

/* Receive normal descriptor (read format) */
#define ETH_RDES0_BUF1AP        0xFFFFFFFF
#define ETH_RDES2_BUF2AP        0xFFFFFFFF
#define ETH_RDES3_OWN           0x80000000
#define ETH_RDES3_IOC           0x40000000
#define ETH_RDES3_BUF2V         0x02000000
#define ETH_RDES3_BUF1V         0x01000000

/* Receive normal descriptor (write-back format) */
#define ETH_RDES0_IVT           0xFFFF0000
#define ETH_RDES0_OVT           0x0000FFFF
#define ETH_RDES1_OPC           0xFFFF0000
#define ETH_RDES1_TD            0x00008000
#define ETH_RDES1_TSA           0x00004000
#define ETH_RDES1_PV            0x00002000
#define ETH_RDES1_PFT           0x00001000
#define ETH_RDES1_PMT           0x00000F00
#define ETH_RDES1_IPCE          0x00000080
#define ETH_RDES1_IPCB          0x00000040
#define ETH_RDES1_IPV6          0x00000020
#define ETH_RDES1_IPV4          0x00000010
#define ETH_RDES1_IPHE          0x00000008
#define ETH_RDES1_PT            0x00000007
#define ETH_RDES2_L3L4FM        0xE0000000
#define ETH_RDES2_L4FM          0x10000000
#define ETH_RDES2_L3FM          0x08000000
#define ETH_RDES2_MADRM         0x07F80000
#define ETH_RDES2_HF            0x00040000
#define ETH_RDES2_DAF           0x00020000
#define ETH_RDES2_SAF           0x00010000
#define ETH_RDES2_VF            0x00008000
#define ETH_RDES2_ARPRN         0x00000400
#define ETH_RDES3_OWN           0x80000000
#define ETH_RDES3_CTXT          0x40000000
#define ETH_RDES3_FD            0x20000000
#define ETH_RDES3_LD            0x10000000
#define ETH_RDES3_RS2V          0x08000000
#define ETH_RDES3_RS1V          0x04000000
#define ETH_RDES3_RS0V          0x02000000
#define ETH_RDES3_CE            0x01000000
#define ETH_RDES3_GP            0x00800000
#define ETH_RDES3_RWT           0x00400000
#define ETH_RDES3_OE            0x00200000
#define ETH_RDES3_RE            0x00100000
#define ETH_RDES3_DE            0x00080000
#define ETH_RDES3_LT            0x00070000
#define ETH_RDES3_ES            0x00008000
#define ETH_RDES3_PL            0x00007FFF

/* Receive context descriptor */
#define ETH_RDES0_RTSL          0xFFFFFFFF
#define ETH_RDES1_RTSH          0xFFFFFFFF
#define ETH_RDES3_OWN           0x80000000
#define ETH_RDES3_CTXT          0x40000000

 //RTL8211F PHY registers
 #define RTL8211F_BMCR                          0x00
 #define RTL8211F_BMSR                          0x01
 #define RTL8211F_PHYID1                        0x02
 #define RTL8211F_PHYID2                        0x03
 #define RTL8211F_ANAR                          0x04
 #define RTL8211F_ANLPAR                        0x05
 #define RTL8211F_ANER                          0x06
 #define RTL8211F_ANNPTR                        0x07
 #define RTL8211F_ANNPRR                        0x08
 #define RTL8211F_GBCR                          0x09
 #define RTL8211F_GBSR                          0x0A
 #define RTL8211F_MMDACR                        0x0D
 #define RTL8211F_MMDAADR                       0x0E
 #define RTL8211F_GBESR                         0x0F
 #define RTL8211F_INER                          0x12
 #define RTL8211F_PHYCR1                        0x18
 #define RTL8211F_PHYCR2                        0x19
 #define RTL8211F_PHYSR                         0x1A
 #define RTL8211F_INSR                          0x1D
 #define RTL8211F_PAGSR                         0x1F
  
 //RTL8211F MMD registers
 #define RTL8211F_PC1R                          0x03, 0x00
 #define RTL8211F_PS1R                          0x03, 0x01
 #define RTL8211F_EEECR                         0x03, 0x14
 #define RTL8211F_EEEWER                        0x03, 0x16
 #define RTL8211F_EEEAR                         0x07, 0x3C
 #define RTL8211F_EEELPAR                       0x07, 0x3D
  
 //Basic Mode Control register
 #define RTL8211F_BMCR_RESET                    0x8000
 #define RTL8211F_BMCR_LOOPBACK                 0x4000
 #define RTL8211F_BMCR_SPEED_SEL_LSB            0x2000
 #define RTL8211F_BMCR_AN_EN                    0x1000
 #define RTL8211F_BMCR_POWER_DOWN               0x0800
 #define RTL8211F_BMCR_ISOLATE                  0x0400
 #define RTL8211F_BMCR_RESTART_AN               0x0200
 #define RTL8211F_BMCR_DUPLEX_MODE              0x0100
 #define RTL8211F_BMCR_COL_TEST                 0x0080
 #define RTL8211F_BMCR_SPEED_SEL_MSB            0x0040
 #define RTL8211F_BMCR_UNI_DIR_EN               0x0020
  
 //Basic Mode Status register
 #define RTL8211F_BMSR_100BT4                   0x8000
 #define RTL8211F_BMSR_100BTX_FD                0x4000
 #define RTL8211F_BMSR_100BTX_HD                0x2000
 #define RTL8211F_BMSR_10BT_FD                  0x1000
 #define RTL8211F_BMSR_10BT_HD                  0x0800
 #define RTL8211F_BMSR_100BT2_FD                0x0400
 #define RTL8211F_BMSR_100BT2_HD                0x0200
 #define RTL8211F_BMSR_EXTENDED_STATUS          0x0100
 #define RTL8211F_BMSR_UNI_DIR_CAPABLE          0x0080
 #define RTL8211F_BMSR_PREAMBLE_SUPPR           0x0040
 #define RTL8211F_BMSR_AN_COMPLETE              0x0020
 #define RTL8211F_BMSR_REMOTE_FAULT             0x0010
 #define RTL8211F_BMSR_AN_CAPABLE               0x0008
 #define RTL8211F_BMSR_LINK_STATUS              0x0004
 #define RTL8211F_BMSR_JABBER_DETECT            0x0002
 #define RTL8211F_BMSR_EXTENDED_CAPABLE         0x0001
  
 //PHY Identifier 1 register
 #define RTL8211F_PHYID1_OUI_MSB                0xFFFF
 #define RTL8211F_PHYID1_OUI_MSB_DEFAULT        0x001C
  
 //PHY Identifier 2 register
 #define RTL8211F_PHYID2_OUI_LSB                0xFC00
 #define RTL8211F_PHYID2_OUI_LSB_DEFAULT        0xC800
 #define RTL8211F_PHYID2_MODEL_NUM              0x03F0
 #define RTL8211F_PHYID2_MODEL_NUM_DEFAULT      0x0110
 #define RTL8211F_PHYID2_REVISION_NUM           0x000F
 #define RTL8211F_PHYID2_REVISION_NUM_DEFAULT   0x0006
  
 //Auto-Negotiation Advertisement register
 #define RTL8211F_ANAR_NEXT_PAGE                0x8000
 #define RTL8211F_ANAR_REMOTE_FAULT             0x2000
 #define RTL8211F_ANAR_ASYM_PAUSE               0x0800
 #define RTL8211F_ANAR_PAUSE                    0x0400
 #define RTL8211F_ANAR_100BT4                   0x0200
 #define RTL8211F_ANAR_100BTX_FD                0x0100
 #define RTL8211F_ANAR_100BTX_HD                0x0080
 #define RTL8211F_ANAR_10BT_FD                  0x0040
 #define RTL8211F_ANAR_10BT_HD                  0x0020
 #define RTL8211F_ANAR_SELECTOR                 0x001F
 #define RTL8211F_ANAR_SELECTOR_DEFAULT         0x0001
  
 //Auto-Negotiation Link Partner Ability register
 #define RTL8211F_ANLPAR_NEXT_PAGE              0x8000
 #define RTL8211F_ANLPAR_ACK                    0x4000
 #define RTL8211F_ANLPAR_REMOTE_FAULT           0x2000
 #define RTL8211F_ANLPAR_ASYM_PAUSE             0x0800
 #define RTL8211F_ANLPAR_PAUSE                  0x0400
 #define RTL8211F_ANLPAR_100BT4                 0x0200
 #define RTL8211F_ANLPAR_100BTX_FD              0x0100
 #define RTL8211F_ANLPAR_100BTX_HD              0x0080
 #define RTL8211F_ANLPAR_10BT_FD                0x0040
 #define RTL8211F_ANLPAR_10BT_HD                0x0020
 #define RTL8211F_ANLPAR_SELECTOR               0x001F
 #define RTL8211F_ANLPAR_SELECTOR_DEFAULT       0x0001
  
 //Auto-Negotiation Expansion register
 #define RTL8211F_ANER_RX_NP_LOCATION_ABLE      0x0040
 #define RTL8211F_ANER_RX_NP_LOCATION           0x0020
 #define RTL8211F_ANER_PAR_DETECT_FAULT         0x0010
 #define RTL8211F_ANER_LP_NEXT_PAGE_ABLE        0x0008
 #define RTL8211F_ANER_NEXT_PAGE_ABLE           0x0004
 #define RTL8211F_ANER_PAGE_RECEIVED            0x0002
 #define RTL8211F_ANER_LP_AN_ABLE               0x0001
  
 //Auto-Negotiation Next Page Transmit register
 #define RTL8211F_ANNPTR_NEXT_PAGE              0x8000
 #define RTL8211F_ANNPTR_MSG_PAGE               0x2000
 #define RTL8211F_ANNPTR_ACK2                   0x1000
 #define RTL8211F_ANNPTR_TOGGLE                 0x0800
 #define RTL8211F_ANNPTR_MESSAGE                0x07FF
  
 //Auto-Negotiation Next Page Receive register
 #define RTL8211F_ANNPRR_NEXT_PAGE              0x8000
 #define RTL8211F_ANNPRR_ACK                    0x4000
 #define RTL8211F_ANNPRR_MSG_PAGE               0x2000
 #define RTL8211F_ANNPRR_ACK2                   0x1000
 #define RTL8211F_ANNPRR_TOGGLE                 0x0800
 #define RTL8211F_ANNPRR_MESSAGE                0x07FF
  
 //1000Base-T Control register
 #define RTL8211F_GBCR_TEST_MODE                0xE000
 #define RTL8211F_GBCR_MS_MAN_CONF_EN           0x1000
 #define RTL8211F_GBCR_MS_MAN_CONF_VAL          0x0800
 #define RTL8211F_GBCR_PORT_TYPE                0x0400
 #define RTL8211F_GBCR_1000BT_FD                0x0200
  
 //1000Base-T Status register
 #define RTL8211F_GBSR_MS_CONF_FAULT            0x8000
 #define RTL8211F_GBSR_MS_CONF_RES              0x4000
 #define RTL8211F_GBSR_LOCAL_RECEIVER_STATUS    0x2000
 #define RTL8211F_GBSR_REMOTE_RECEIVER_STATUS   0x1000
 #define RTL8211F_GBSR_LP_1000BT_FD             0x0800
 #define RTL8211F_GBSR_LP_1000BT_HD             0x0400
 #define RTL8211F_GBSR_IDLE_ERR_COUNT           0x00FF
  
 //MMD Access Control register
 #define RTL8211F_MMDACR_FUNC                   0xC000
 #define RTL8211F_MMDACR_FUNC_ADDR              0x0000
 #define RTL8211F_MMDACR_FUNC_DATA_NO_POST_INC  0x4000
 #define RTL8211F_MMDACR_FUNC_DATA_POST_INC_RW  0x8000
 #define RTL8211F_MMDACR_FUNC_DATA_POST_INC_W   0xC000
 #define RTL8211F_MMDACR_DEVAD                  0x001F
  
 //1000Base-T Extended Status register
 #define RTL8211F_GBESR_1000BX_FD               0x8000
 #define RTL8211F_GBESR_1000BX_HD               0x4000
 #define RTL8211F_GBESR_1000BT_FD               0x2000
 #define RTL8211F_GBESR_1000BT_HD               0x1000
  
 //Interrupt Enable register
 #define RTL8211F_INER_JABBER                   0x0400
 #define RTL8211F_INER_ALDPS_STATE              0x0200
 #define RTL8211F_INER_PME                      0x0080
 #define RTL8211F_INER_PHY_REG_ACCESS           0x0020
 #define RTL8211F_INER_LINK_STATUS              0x0010
 #define RTL8211F_INER_AN_COMPLETE              0x0008
 #define RTL8211F_INER_PAGE_RECEIVED            0x0004
 #define RTL8211F_INER_AN_ERROR                 0x0001
  
 //PHY Specific Control 1 register
 #define RTL8211F_PHYCR1_PHYAD_0_EN             0x2000
 #define RTL8211F_PHYCR1_MDI_MODE_MANUAL_CONFIG 0x0200
 #define RTL8211F_PHYCR1_MDI_MODE               0x0100
 #define RTL8211F_PHYCR1_TX_CRS_EN              0x0080
 #define RTL8211F_PHYCR1_PHYAD_NON_ZERO_DETECT  0x0040
 #define RTL8211F_PHYCR1_PREAMBLE_CHECK_EN      0x0010
 #define RTL8211F_PHYCR1_JABBER_DETECT_EN       0x0008
 #define RTL8211F_PHYCR1_ALDPS_EN               0x0004
  
 //PHY Specific Control 2 register
 #define RTL8211F_PHYCR2_CLKOUT_FREQ_SEL        0x0800
 #define RTL8211F_PHYCR2_CLKOUT_SSC_EN          0x0080
 #define RTL8211F_PHYCR2_RXC_SSC_EN             0x0008
 #define RTL8211F_PHYCR2_RXC_EN                 0x0002
 #define RTL8211F_PHYCR2_CLKOUT_EN              0x0001
  
 //PHY Specific Status register
 #define RTL8211F_PHYSR_ALDPS_STATE             0x4000
 #define RTL8211F_PHYSR_MDI_PLUG                0x2000
 #define RTL8211F_PHYSR_NWAY_EN                 0x1000
 #define RTL8211F_PHYSR_MASTER_MODE             0x0800
 #define RTL8211F_PHYSR_EEE_CAPABLE             0x0100
 #define RTL8211F_PHYSR_RX_FLOW_EN              0x0080
 #define RTL8211F_PHYSR_TX_FLOW_EN              0x0040
 #define RTL8211F_PHYSR_SPEED                   0x0030
 #define RTL8211F_PHYSR_SPEED_10MBPS            0x0000
 #define RTL8211F_PHYSR_SPEED_100MBPS           0x0010
 #define RTL8211F_PHYSR_SPEED_1000MBPS          0x0020
 #define RTL8211F_PHYSR_DUPLEX                  0x0008
 #define RTL8211F_PHYSR_LINK                    0x0004
 #define RTL8211F_PHYSR_MDI_CROSSOVER_STATUS    0x0002
 #define RTL8211F_PHYSR_JABBER                  0x0001
  
 //Interrupt Status register
 #define RTL8211F_INSR_JABBER                   0x0400
 #define RTL8211F_INSR_ALDPS_STATE              0x0200
 #define RTL8211F_INSR_PME                      0x0080
 #define RTL8211F_INSR_PHY_REG_ACCESS           0x0020
 #define RTL8211F_INSR_LINK_STATUS              0x0010
 #define RTL8211F_INSR_AN_COMPLETE              0x0008
 #define RTL8211F_INSR_PAGE_RECEIVED            0x0004
 #define RTL8211F_INSR_AN_ERROR                 0x0001
  
 //Page Select register
 #define RTL8211F_PAGSR_PAGE_SEL                0x0007
  
 //PCS Control 1 register
 #define RTL8211F_PC1R_CLK_STOP_EN              0x0400
  
 //PCS Status 1 register
 #define RTL8211F_PS1R_TX_LPI_RCVD              0x0800
 #define RTL8211F_PS1R_RX_LPI_RCVD              0x0400
 #define RTL8211F_PS1R_TX_LPI_IND               0x0200
 #define RTL8211F_PS1R_RX_LPI_IND               0x0100
 #define RTL8211F_PS1R_CLK_STOP_CAPABLE         0x0040
  
 //EEE Capability register
 #define RTL8211F_EEECR_1000BT_EEE              0x0004
 #define RTL8211F_EEECR_100BTX_EEE              0x0002
  
 //EEE Advertisement register
 #define RTL8211F_EEEAR_1000BT_EEE              0x0004
 #define RTL8211F_EEEAR_100BTX_EEE              0x0002
  
 //EEE Link Partner Ability register
 #define RTL8211F_EEELPAR_LP_1000BT_EEE         0x0004
 #define RTL8211F_EEELPAR_LP_100BTX_EEE         0x0002

#ifdef __cplusplus
}
#endif

#endif
