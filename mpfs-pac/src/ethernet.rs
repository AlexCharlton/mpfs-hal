// Network Control register bit definitions
pub const GEM_IFG_EATS_QAV_CREDIT: u32 = 1 << 30;
pub const GEM_TWO_PT_FIVE_GIG: u32 = 1 << 29;
pub const GEM_SEL_MII_ON_RGMII: u32 = 1 << 28;
pub const GEM_OSS_CORRECTION_FIELD: u32 = 1 << 27;
pub const GEM_EXT_RXQ_SEL_EN: u32 = 1 << 26;
pub const GEM_PFC_CTRL: u32 = 1 << 25;
pub const GEM_ONE_STEP_SYNC_MODE: u32 = 1 << 24;
pub const GEM_EXT_TSU_PORT_ENABLE: u32 = 1 << 23;
pub const GEM_STORE_UDP_OFFSET: u32 = 1 << 22;
pub const GEM_ALT_SGMII_MODE: u32 = 1 << 21;
pub const GEM_PTP_UNICAST_ENA: u32 = 1 << 20;
pub const GEM_TX_LPI_EN: u32 = 1 << 19;
pub const GEM_FLUSH_RX_PKT_PCLK: u32 = 1 << 18;
pub const GEM_TRANSMIT_PFC_PRIORITY_BASED_PAUSE_FRAME: u32 = 1 << 17;
pub const GEM_PFC_ENABLE: u32 = 1 << 16;
pub const GEM_STORE_RX_TS: u32 = 1 << 15;
pub const GEM_TX_PAUSE_FRAME_ZERO: u32 = 1 << 12;
pub const GEM_TX_PAUSE_FRAME_REQ: u32 = 1 << 11;
pub const GEM_TRANSMIT_HALT: u32 = 1 << 10;
pub const GEM_TRANSMIT_START: u32 = 1 << 9;
pub const GEM_BACK_PRESSURE: u32 = 1 << 8;
pub const GEM_STATS_WRITE_EN: u32 = 1 << 7;
pub const GEM_INC_ALL_STATS_REGS: u32 = 1 << 6;
pub const GEM_CLEAR_ALL_STATS_REGS: u32 = 1 << 5;
pub const GEM_MAN_PORT_EN: u32 = 1 << 4;
pub const GEM_ENABLE_TRANSMIT: u32 = 1 << 3;
pub const GEM_ENABLE_RECEIVE: u32 = 1 << 2;
pub const GEM_LOOPBACK_LOCAL: u32 = 1 << 1;
pub const GEM_LOOPBACK: u32 = 1 << 0;

/* General MAC Network Configuration register bit definitions */
/* eMAC Network Configuration register bit definitions */

pub const GEM_UNI_DIRECTION_ENABLE: u32 = 1 << 31;
pub const GEM_IGNORE_IPG_RX_ER: u32 = 1 << 30;
pub const GEM_NSP_CHANGE: u32 = 1 << 29;
pub const GEM_IPG_STRETCH_ENABLE: u32 = 1 << 28;
pub const GEM_SGMII_MODE_ENABLE: u32 = 1 << 27;
pub const GEM_IGNORE_RX_FCS: u32 = 1 << 26;
pub const GEM_EN_HALF_DUPLEX_RX: u32 = 1 << 25;
pub const GEM_RECEIVE_CHECKSUM_OFFLOAD_ENABLE: u32 = 1 << 24;
pub const GEM_DISABLE_COPY_OF_PAUSE_FRAMES: u32 = 1 << 23;
pub const GEM_DATA_BUS_WIDTH: u32 = (1 << 21) | (1 << 22);
pub const GEM_MDC_CLOCK_DIVISOR: u32 = (1 << 18) | (1 << 19) | (1 << 20);
pub const GEM_FCS_REMOVE: u32 = 1 << 17;
pub const GEM_LENGTH_FIELD_ERROR_FRAME_DISCARD: u32 = 1 << 16;
pub const GEM_RECEIVE_BUFFER_OFFSET: u32 = (1 << 14) | (1 << 15);
pub const GEM_PAUSE_ENABLE: u32 = 1 << 13;
pub const GEM_RETRY_TEST: u32 = 1 << 12;
pub const GEM_PCS_SELECT: u32 = 1 << 11;
pub const GEM_GIGABIT_MODE_ENABLE: u32 = 1 << 10;
pub const GEM_EXTERNAL_ADDRESS_MATCH_ENABLE: u32 = 1 << 9;
pub const GEM_RECEIVE_1536_BYTE_FRAMES: u32 = 1 << 8;
pub const GEM_UNICAST_HASH_ENABLE: u32 = 1 << 7;
pub const GEM_MULTICAST_HASH_ENABLE: u32 = 1 << 6;
pub const GEM_NO_BROADCAST: u32 = 1 << 5;
pub const GEM_COPY_ALL_FRAMES: u32 = 1 << 4;
pub const GEM_JUMBO_FRAMES: u32 = 1 << 3;
pub const GEM_DISCARD_NON_VLAN_FRAMES: u32 = 1 << 2;
pub const GEM_FULL_DUPLEX: u32 = 1 << 1;
pub const GEM_SPEED: u32 = 1 << 0;

pub const GEM_DATA_BUS_WIDTH_SHIFT: u32 = 21;
pub const GEM_MDC_CLOCK_DIVISOR_SHIFT: u32 = 18;
pub const GEM_MDC_CLOCK_DIVISOR_MASK: u32 = 0b111;
pub const GEM_RECEIVE_BUFFER_OFFSET_SHIFT: u32 = 14;

/* General MAC Network Status register bit definitions */
/* eMAC Network Status register bit definitions */

pub const GEM_LPI_INDICATE_PCLK: u32 = 1 << 7;
pub const GEM_PFC_NEGOTIATE_PCLK: u32 = 1 << 6;
pub const GEM_MAC_PAUSE_TX_EN: u32 = 1 << 5;
pub const GEM_MAC_PAUSE_RX_EN: u32 = 1 << 4;
pub const GEM_MAC_FULL_DUPLEX: u32 = 1 << 3;
pub const GEM_MAN_DONE: u32 = 1 << 2;
pub const GEM_MDIO_IN: u32 = 1 << 1;
pub const GEM_PCS_LINK_STATE: u32 = 1 << 0;

/* General MAC User IO register bit definitions */

pub const GEM_CODEGROUP_BYPASS: u32 = 1 << 5;
pub const GEM_COMMA_BYPASS: u32 = 1 << 4;
pub const GEM_TSU_CLK_SOURCE: u32 = 1 << 0;

/* General MAC DMA Config register bit definitions */
/* eMAC DMA Config register bit definitions */

pub const GEM_DMA_ADDR_BUS_WIDTH_1: u32 = 1 << 30;
pub const GEM_TX_BD_EXTENDED_MODE_EN: u32 = 1 << 29;
pub const GEM_RX_BD_EXTENDED_MODE_EN: u32 = 1 << 28;
pub const GEM_FORCE_MAX_AMBA_BURST_TX: u32 = 1 << 26;
pub const GEM_FORCE_MAX_AMBA_BURST_RX: u32 = 1 << 25;
pub const GEM_FORCE_DISCARD_ON_ERR: u32 = 1 << 24;
pub const GEM_RX_BUF_SIZE: u32 = 0xFF << 16;
pub const GEM_CRC_ERROR_REPORT: u32 = 1 << 13;
pub const GEM_INFINITE_LAST_DBUF_SIZE_EN: u32 = 1 << 12;
pub const GEM_TX_PBUF_TCP_EN: u32 = 1 << 11;
pub const GEM_TX_PBUF_SIZE: u32 = 1 << 10;
pub const GEM_RX_PBUF_SIZE: u32 = (1 << 8) | (1 << 9);
pub const GEM_ENDIAN_SWAP_PACKET: u32 = 1 << 7;
pub const GEM_ENDIAN_SWAP_MANAGEMENT: u32 = 1 << 6;
pub const GEM_HDR_DATA_SPLITTING_EN: u32 = 1 << 5;
pub const GEM_AMBA_BURST_LENGTH: u32 = 0b11111;

pub const GEM_RX_BUF_SIZE_SHIFT: u32 = 16;
pub const GEM_RX_PBUF_SIZE_SHIFT: u32 = 8;

/* General MAC Transmit Status register bit definitions */
/* eMAC Transmit Status register bit definitions */

pub const GEM_TX_DMA_LOCKUP_DETECTED: u32 = 1 << 10;
pub const GEM_TX_MAC_LOCKUP_DETECTED: u32 = 1 << 9;
pub const GEM_TX_RESP_NOT_OK: u32 = 1 << 8;
pub const GEM_LATE_COLLISION_OCCURRED: u32 = 1 << 7;
pub const GEM_STAT_TRANSMIT_UNDER_RUN: u32 = 1 << 6;
pub const GEM_STAT_TRANSMIT_COMPLETE: u32 = 1 << 5;
pub const GEM_STAT_AMBA_ERROR: u32 = 1 << 4;
pub const GEM_TRANSMIT_GO: u32 = 1 << 3;
pub const GEM_RETRY_LIMIT_EXCEEDED: u32 = 1 << 2;
pub const GEM_COLLISION_OCCURRED: u32 = 1 << 1;
pub const GEM_USED_BIT_READ: u32 = 1 << 0;

/* General MAC Receive Queue Pointer register bit definitions */
/* General MAC Receive Queue 1 Pointer register bit definitions */
/* General MAC Receive Queue 2 Pointer register bit definitions */
/* General MAC Receive Queue 3 Pointer register bit definitions */
/* eMAC Receive Queue Pointer register bit definitions */

pub const GEM_DMA_RX_Q_PTR: u32 = !((1 << 0) | (1 << 1));
pub const GEM_DMA_RX_DIS_Q: u32 = 1 << 0;

/* General MAC Transmit Queue Pointer register bit definitions */
/* General MAC Transmit Queue 1 Pointer register bit definitions */
/* General MAC Transmit Queue 2 Pointer register bit definitions */
/* General MAC Transmit Queue 3 Pointer register bit definitions */
/* eMAC Transmit Queue Pointer register bit definitions */

pub const GEM_DMA_TX_Q_PTR: u32 = !((1 << 0) | (1 << 1));
pub const GEM_DMA_TX_DIS_Q: u32 = 1 << 0;

/* General MAC Receive Status register bit definitions */
/* eMAC Receive Status register bit definitions */

pub const GEM_RX_DMA_LOCKUP_DETECTED: u32 = 1 << 5;
pub const GEM_RX_MAC_LOCKUP_DETECTED: u32 = 1 << 4;
pub const GEM_RX_RESP_NOT_OK: u32 = 1 << 3;
pub const GEM_RECEIVE_OVERRUN: u32 = 1 << 2;
pub const GEM_FRAME_RECEIVED: u32 = 1 << 1;
pub const GEM_BUFFER_NOT_AVAILABLE: u32 = 1 << 0;

/* General MAC Interrupt Status register bit definitions */
/* General MAC Interrupt Enable register bit definitions */
/* General MAC Interrupt Disable register bit definitions */
/* General MAC Interrupt Mask register bit definitions */
/* General MAC Priority Queue 1 Interrupt Status register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 2 Interrupt Status register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 3 Interrupt Status register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 1 Interrupt Enable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 2 Interrupt Enable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 3 Interrupt Enable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 1 Interrupt Disable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 2 Interrupt Disable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 3 Interrupt Disable register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 1 Interrupt Mask register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 2 Interrupt Mask register bit definitions  - b01 to b11 only */
/* General MAC Priority Queue 3 Interrupt Mask register bit definitions  - b01 to b11 only */
/* eMAC Interrupt Status register bit definitions */
/* eMAC Interrupt Enable register bit definitions */
/* eMAC Interrupt Disable register bit definitions */
/* eMAC Interrupt Mask register bit definitions */

pub const GEM_TX_LOCKUP_DETECTED: u32 = 1 << 31;
pub const GEM_RX_LOCKUP_DETECTED: u32 = 1 << 30;
pub const GEM_TSU_TIMER_COMPARISON_INTERRUPT: u32 = 1 << 29;
pub const GEM_WOL_INTERRUPT: u32 = 1 << 28;
pub const GEM_RX_LPI_INDICATION_STATUS_BIT_CHANGE: u32 = 1 << 27;
pub const GEM_TSU_SECONDS_REGISTER_INCREMENT: u32 = 1 << 26;
pub const GEM_PTP_PDELAY_RESP_FRAME_TRANSMITTED: u32 = 1 << 25;
pub const GEM_PTP_PDELAY_REQ_FRAME_TRANSMITTED: u32 = 1 << 24;
pub const GEM_PTP_PDELAY_RESP_FRAME_RECEIVED: u32 = 1 << 23;
pub const GEM_PTP_PDELAY_REQ_FRAME_RECEIVED: u32 = 1 << 22;
pub const GEM_PTP_SYNC_FRAME_TRANSMITTED: u32 = 1 << 21;
pub const GEM_PTP_DELAY_REQ_FRAME_TRANSMITTED: u32 = 1 << 20;
pub const GEM_PTP_SYNC_FRAME_RECEIVED: u32 = 1 << 19;
pub const GEM_PTP_DELAY_REQ_FRAME_RECEIVED: u32 = 1 << 18;
pub const GEM_PCS_LINK_PARTNER_PAGE_RECEIVED: u32 = 1 << 17;
pub const GEM_PCS_AUTO_NEGOTIATION_COMPLETE: u32 = 1 << 16;
pub const GEM_EXTERNAL_INTERRUPT: u32 = 1 << 15;
pub const GEM_PAUSE_FRAME_TRANSMITTED: u32 = 1 << 14;
pub const GEM_PAUSE_TIME_ELAPSED: u32 = 1 << 13;
pub const GEM_PAUSE_FRAME_WITH_NON_0_PAUSE_QUANTUM_RX: u32 = 1 << 12;
pub const GEM_RESP_NOT_OK_INT: u32 = 1 << 11;
pub const GEM_RECEIVE_OVERRUN_INT: u32 = 1 << 10;
pub const GEM_LINK_CHANGE: u32 = 1 << 9;
pub const GEM_TRANSMIT_COMPLETE: u32 = 1 << 7;
pub const GEM_AMBA_ERROR: u32 = 1 << 6;
pub const GEM_RETRY_LIMIT_EXCEEDED_OR_LATE_COLLISION: u32 = 1 << 5;
pub const GEM_TRANSMIT_UNDER_RUN: u32 = 1 << 4;
pub const GEM_TX_USED_BIT_READ: u32 = 1 << 3;
pub const GEM_RX_USED_BIT_READ: u32 = 1 << 2;
pub const GEM_RECEIVE_COMPLETE: u32 = 1 << 1;
pub const GEM_MANAGEMENT_FRAME_SENT: u32 = 1 << 0;

/*
 * General MAC Fatal or Non Fatal Interrupt register bit definitions
 * Note bits 0 to 15 are as per interrupt mask etc registers above.
 */

pub const GEM_LOCKUP_DETECTED_INT_TYPE: u32 = 1 << 22;
pub const GEM_TSU_TIMER_COMPARISON_INTERRUPT_INT_TYPE: u32 = 1 << 21;
pub const GEM_WOL_INTERRUPT_INT_TYPE: u32 = 1 << 20;
pub const GEM_RECEIVE_LPI_INT_TYPE: u32 = 1 << 19;
pub const GEM_TSU_SECONDS_REGISTER_INCREMENT_INT_TYPE: u32 = 1 << 18;
pub const GEM_PTP_FRAME_RECEIVED_INT_TYPE: u32 = 1 << 17;
pub const GEM_PCS_INT_TYPE: u32 = 1 << 16;

/* General MAC Phy Management register bit definitions */
/* eMAC Phy Management register bit definitions */

pub const GEM_WRITE0: u32 = 1 << 31;
pub const GEM_WRITE1: u32 = 1 << 30;
pub const GEM_OPERATION: u32 = (1 << 28) | (1 << 29);
pub const GEM_PHY_ADDRESS: u32 = 0b11111 << 23;
pub const GEM_REGISTER_ADDRESS: u32 = 0b11111 << 18;
pub const GEM_WRITE10: u32 = (1 << 16) | (1 << 17);
pub const GEM_PHY_WRITE_READ_DATA: u32 = 0xFFFF;

pub const GEM_PHY_OP_CL22_WRITE: u32 = 1;
pub const GEM_PHY_OP_CL22_READ: u32 = 2;

pub const GEM_PHY_OP_CL45_ADDRESS: u32 = 0;
pub const GEM_PHY_OP_CL45_WRITE: u32 = 1;
pub const GEM_PHY_OP_CL45_POST_READ_INC: u32 = 2;
pub const GEM_PHY_OP_CL45_READ: u32 = 3;

pub const GEM_OPERATION_SHIFT: u32 = 28;
pub const GEM_PHY_ADDRESS_SHIFT: u32 = 23;
pub const GEM_REGISTER_ADDRESS_SHIFT: u32 = 18;
pub const GEM_WRITE10_SHIFT: u32 = 16;

/* General MAC Pause Time register bit definitions */
/* General MAC Transmit Pause Time register bit definitions */
/* eMAC Pause Time register bit definitions */
/* eMAC Transmit Pause Time register bit definitions */

pub const GEM_QUANTUM: u32 = 0xFFFF;

/* General MAC PBuff TX Cutthru register bit definitions */
/* General MAC PBuff RX Cutthru register bit definitions */
/* eMAC PBuff TX Cutthru register bit definitions */
/* eMAC PBuff RX Cutthru register bit definitions */

pub const GEM_DMA_CUTTHRU: u32 = 1 << 31;
pub const GEM_DMA_TX_CUTTHRU_THRESHOLD: u32 = 0b11111111111;
pub const GEM_DMA_RX_CUTTHRU_THRESHOLD: u32 = 0b1111111111;
pub const GEM_DMA_EMAC_CUTTHRU_THRESHOLD: u32 = 0b111111111;

/* General MAC AXI Max Pipeline register bit definitions */
/* eMAC AXI Max Pipeline register bit definitions */

pub const GEM_USE_AW2B_FILL: u32 = 1 << 16;
pub const GEM_AW2W_MAX_PIPELINE: u32 = 0xFF << 8;
pub const GEM_AR2R_MAX_PIPELINE: u32 = 0xFF;

/* General MAC Int Moderation register bit definitions */
/* eMAC Int Moderation register bit definitions */

pub const GEM_TX_INT_MODERATION: u32 = 0xFF << 16;
pub const GEM_RX_INT_MODERATION: u32 = 0xFF;

/* General MAC Sys Wake Time register bit definitions */
/* eMAC Sys Wake Time register bit definitions */

pub const GEM_SYS_WAKE_TIME: u32 = 0xFFFF;

/* General MAC Lockup Config register bit definitions */
/* General RX MAC Lockup Time register bit definitions */
/* eMAC Lockup Config register bit definitions */
/* RX eMAC Lockup Time register bit definitions */

pub const GEM_TX_DMA_LOCKUP_MON_EN: u32 = 1 << 31;
pub const GEM_TX_MAC_LOCKUP_MON_EN: u32 = 1 << 30;
pub const GEM_RX_DMA_LOCKUP_MON_EN: u32 = 1 << 29;
pub const GEM_RX_MAC_LOCKUP_MON_EN: u32 = 1 << 28;
pub const GEM_LOCKUP_RECOVERY_EN: u32 = 1 << 27;
pub const GEM_LOCKUP_TIME: u32 = 0xFFFF;

/* General MAC Specific Address 1 Top register bit definitions */
/* General MAC Specific Address 2 Top register bit definitions */
/* General MAC Specific Address 3 Top register bit definitions */
/* General MAC Specific Address 4 Top register bit definitions */
/* eMAC Specific Address 1 Top register bit definitions */
/* eMAC Specific Address 2 Top register bit definitions */
/* eMAC Specific Address 3 Top register bit definitions */
/* eMAC Specific Address 4 Top register bit definitions */

pub const GEM_FILTER_BYTE_MASK: u32 = 0b111111 << 24;
pub const GEM_FILTER_TYPE: u32 = 1 << 16;
pub const GEM_SPEC_ADDRESS: u32 = 0xFFFF;

/* General MAC Specific Address Type 1 register bit definitions */
/* General MAC Specific Address Type 2 register bit definitions */
/* General MAC Specific Address Type 3 register bit definitions */
/* General MAC Specific Address Type 4 register bit definitions */
/* eMAC Specific Address Type 1 register bit definitions */
/* eMAC Specific Address Type 2 register bit definitions */
/* eMAC Specific Address Type 3 register bit definitions */
/* eMAC Specific Address Type 4 register bit definitions */

pub const GEM_ENABLE_COPY: u32 = 1 << 31;
pub const GEM_SPEC_ADDR_MATCH: u32 = 0xFFFF;

/* General MAC Wake On LAN register bit definitions */
/* eMAC Wake On LAN register bit definitions */

pub const GEM_WOL_MULTICAST_HASH: u32 = 1 << 19;
pub const GEM_WOL_SPEC_ADDRESS_1: u32 = 1 << 18;
pub const GEM_WOL_ARP_REQUEST: u32 = 1 << 17;
pub const GEM_WOL_MAGIC_PACKET: u32 = 1 << 16;
pub const GEM_WOL_ADDRESS: u32 = 0xFFFF;

/* General MAC Stretch Ratio register bit definitions */
/* eMAC Stretch Ratio register bit definitions */

pub const GEM_IPG_STRETCH: u32 = 0xFFFF;
pub const GEM_IPG_STRETCH_DIV: u32 = 0xFF << 8;
pub const GEM_IPG_STRETCH_MUL: u32 = 0xFF;

pub const GEM_IPG_STRETCH_DIV_MASK: u32 = 0xFF;
pub const GEM_IPG_STRETCH_DIV_SHIFT: u32 = 8;
pub const GEM_IPG_STRETCH_MUL_MASK: u32 = 0xFF;

/* General MAC Stacked VLAN register bit definitions */
/* eMAC Stacked VLAN register bit definitions */

pub const GEM_ENABLE_PROCESSING: u32 = 1 << 31;
pub const GEM_VLAN_MATCH: u32 = 0xFFFF;
pub const GEM_VLAN_C_TAG: u32 = 0x8100;
pub const GEM_VLAN_S_TAG: u32 = 0x88A8;

/* Valid EtherTypes including VLAN tags must be bigger than this */
pub const GEM_VLAN_ETHERTYPE_MIN: u32 = 1536;
pub const GEM_VLAN_NO_STACK: u32 = 0;

/* General MAC Transmit PFC Pause register bit definitions */
/* eMAC Transmit PFC Pause register bit definitions */

pub const GEM_VECTOR: u32 = 0xFF << 8;
pub const GEM_VECTOR_ENABLE: u32 = 0xFF;

/* General MAC Specific Address Type 1 Mask register bit definitions */
/* eMAC Specific Address Type 1 Mask register bit definitions */

pub const GEM_SPEC_ADDR_MASK: u32 = 0xFFFF;

/* General MAC Receive DMA Data Buffer Address Mask register bit definitions */
/* eMAC Receive DMA Data Buffer Address Mask register bit definitions */

pub const GEM_DMA_DBUF_ADDR_MASK_VALUE: u32 = 0b1111 << 28;
pub const GEM_DMA_DBUF_ADDR_MASK_ENABLE: u32 = 0b1111;

/* General MAC TSU timer comparison value nanosecond register bit definitions */
/* eMAC TSU timer comparison value nanosecond register bit definitions */

pub const GEM_NSEC_COMPARISON_VALUE: u32 = 0b1111111111111111111111;

/* General MAC TSU timer comparison value seconds 47:32 register bit definitions */
/* General MAC PTP Event Frame Transmitted Seconds Register 47:32 register bit definitions */
/* General MAC PTP Event Frame Received Seconds Register 47:32 register bit definitions */
/* General MAC PTP Peer Event Frame Transmitted Seconds Register 47:32 register bit definitions */
/* General MAC PTP Peer Event Frame Received Seconds Register 47:32 register bit definitions */
/* eMAC TSU timer comparison value seconds 47:32 register bit definitions */
/* eMAC PTP Event Frame Transmitted Seconds Register 47:32 register bit definitions */
/* eMAC PTP Event Frame Received Seconds Register 47:32 register bit definitions */
/* eMAC PTP Peer Event Frame Transmitted Seconds Register 47:32 register bit definitions */
/* eMAC PTP Peer Event Frame Received Seconds Register 47:32 register bit definitions */

pub const GEM_SEC_VALUE_UPPER: u32 = 0xFFFF;

/* General MAC DP RAM Fill Debug register bit definitions */
/* eMAC DP RAM Fill Debug register bit definitions */

pub const GEM_DMA_TX_RX_FILL_LEVEL: u32 = 0xFFFF << 16;
pub const GEM_DMA_TX_Q_FILL_LEVEL_SELECT: u32 = 0b1111 << 4;
pub const GEM_DMA_TX_RX_FILL_LEVEL_SELECT: u32 = 1 << 0;

/* General MAC Revision register bit definitions */
/* eMAC Revision register bit definitions */

pub const GEM_FIX_NUMBER: u32 = 0b1111 << 24;
pub const GEM_MODULE_IDENTIFICATION_NUMBER: u32 = 0xFFF << 16;
pub const GEM_MODULE_REVISION: u32 = 0xFFFF;

/* General MAC Octets Transmitted Top register bit definitions */
/* General MAC Octets Received Top register bit definitions */
/* eMAC Octets Transmitted Top register bit definitions */
/* eMAC Octets Received Top register bit definitions */

pub const GEM_UPPER_BITS_OF_48: u32 = 0xFFFF;

/* General MAC Pause Frames Transmitted register bit definitions */
/* General MAC Pause Frames Received register bit definitions */
/* eMAC Pause Frames Transmitted register bit definitions */
/* eMAC Pause Frames Received register bit definitions */

pub const GEM_FRAME_COUNT: u32 = 0xFFFF;

/* General MAC Transmit Underruns register bit definitions */
/* eMAC Transmit Underruns register bit definitions */

pub const GEM_UNDERRUN_COUNT: u32 = 0b1111111111;

/* General MAC Single Collision register bit definitions */
/* General MAC Multiple Collisions register bit definitions */
/* eMAC Single Collision register bit definitions */
/* eMAC Multiple Collisions register bit definitions */

pub const GEM_SM_COLLISION_COUNT: u32 = 0b111111111111111111;

/* General MAC Late Collisions register bit definitions */
/* eMAC Late Collisions register bit definitions */

pub const GEM_LATE_COLLISION_COUNT: u32 = 0b1111111111;

/* General MAC Deferred Frames register bit definitions */
/* eMAC Deferred Frames register bit definitions */

pub const GEM_DEFERRED_FRAMES_COUNT: u32 = 0b111111111111111111;

/* General MAC CRS Errors register bit definitions */
/* eMAC CRS Errors register bit definitions */

pub const GEM_CRS_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Undersize Frames Received register bit definitions */
/* eMAC Undersize Frames Received register bit definitions */

pub const GEM_RUNT_FRAME_COUNT: u32 = 0b1111111111;

/* General MAC Oversize Frames Received register bit definitions */
/* eMAC Oversize Frames Received register bit definitions */

pub const GEM_OVERSIZE_FRAME_COUNT: u32 = 0b1111111111;

/* General MAC Jabbers Received register bit definitions */
/* eMAC Jabbers Received register bit definitions */

pub const GEM_JABBER_COUNT: u32 = 0b1111111111;

/* General MAC FCS Error register bit definitions */
/* eMAC FCS Error register bit definitions */

pub const GEM_FCS_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Length Field Frame Errors register bit definitions */
/* eMAC Length Field Frame Errors register bit definitions */

pub const GEM_LENGTH_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Receive Symbol Errors register bit definitions */
/* eMAC Receive Symbol Errors register bit definitions */

pub const GEM_SYMBOL_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Receive Alignment Errors register bit definitions */
/* eMAC Receive Alignment Errors register bit definitions */

pub const GEM_ALIGNMENT_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Receive Resource Error register bit definitions */
/* eMAC Receive Resource Error register bit definitions */

pub const GEM_RESOURCE_ERROR_COUNT: u32 = 0b1111111111;

/* General MAC Receive Overrun register bit definitions */
/* eMAC Receive Overrun register bit definitions */

pub const GEM_OVERRUN_COUNT: u32 = 0b1111111111;

/* General MAC IP Checksum Error register bit definitions */
/* General MAC TCP Checksum Error register bit definitions */
/* General MAC UDP Checksum Error register bit definitions */
/* eMAC IP Checksum Error register bit definitions */
/* eMAC TCP Checksum Error register bit definitions */
/* eMAC UDP Checksum Error register bit definitions */

pub const GEM_IP_CHECKSUM_ERROR_COUNT: u32 = 0xFF;

/* General MAC Auto Flushed Packets register bit definitions */
/* eMAC Auto Flushed Packets register bit definitions */

pub const GEM_AUTO_FLUSHED_COUNT: u32 = 0xFFFF;

/* General MAC TSU Timer Increment Sub Nanoseconds register bit definitions */
/* eMAC TSU Timer Increment Sub Nanoseconds register bit definitions */

pub const GEM_SUB_NS_INCR_LSB: u32 = 0xFF << 24;
pub const GEM_SUB_NS_INCR: u32 = 0xFFFF;

/* General MAC TSU Timer Seconds MSB register bit definitions */
/* General MAC TSU Strobe Seconds MSB register bit definitions */

pub const GEM_TSU_SECONDS_MSB: u32 = 0xFFFF;

/* General MAC TSU Timer Sync Strobe Nanoseconds register bit definitions */
/* General MAC TSU Timer Nanoseconds register bit definitions */
/* General MAC TSU Timer Adjust register bit definitions */
/* General MAC PTP Event Frame Transmitted Nanoseconds register bit definitions */
/* General MAC PTP Event Frame Received Nanoseconds register bit definitions */
/* General MAC PTP Peer Event Frame Transmitted Nanoseconds register bit definitions */
/* General MAC PTP Peer Event Frame Received Nanoseconds register bit definitions */
/* eMAC TSU Timer Sync Strobe Nanoseconds register bit definitions */
/* eMAC TSU Timer Nanoseconds register bit definitions */
/* eMAC TSU Timer Adjust register bit definitions */
/* eMAC PTP Event Frame Transmitted Nanoseconds register bit definitions */
/* eMAC PTP Event Frame Received Nanoseconds register bit definitions */
/* eMAC PTP Peer Event Frame Transmitted Nanoseconds register bit definitions */
/* eMAC PTP Peer Event Frame Received Nanoseconds register bit definitions */

pub const GEM_ADD_SUBTRACT: u32 = 1 << 31; /* Adjust register only... */
pub const GEM_TSU_NANOSECONDS: u32 = 0b111111111111111111111111111111;

/* General MAC TSU Timer Adjust register bit definitions */
/* eMAC TSU Timer Adjust register bit definitions */

pub const GEM_NUM_INCS: u32 = 0xFF << 16;
pub const GEM_ALT_NS_INC: u32 = 0xFF << 8;
pub const GEM_NS_INCREMENT: u32 = 0xFF;

/* General MAC PCS Control register bit definitions */

pub const GEM_PCS_SOFTWARE_RESET: u32 = 1 << 15;
pub const GEM_LOOPBACK_MODE: u32 = 1 << 14;
pub const GEM_SPEED_SELECT_BIT_1: u32 = 1 << 13;
pub const GEM_ENABLE_AUTO_NEG: u32 = 1 << 12;
pub const GEM_RESTART_AUTO_NEG: u32 = 1 << 9;
pub const GEM_MAC_DUPLEX_STATE: u32 = 1 << 8;
pub const GEM_COLLISION_TEST: u32 = 1 << 7;
pub const GEM_SPEED_SELECT_BIT_0: u32 = 1 << 6;

/* General MAC PCS Status register bit definitions */

pub const GEM_BASE_100_T4: u32 = 1 << 15;
pub const GEM_BASE_100_X_FULL_DUPLEX: u32 = 1 << 14;
pub const GEM_BASE_100_X_HALF_DUPLEX: u32 = 1 << 13;
pub const GEM_MBPS_10_FULL_DUPLEX: u32 = 1 << 12;
pub const GEM_MBPS_10_HALF_DUPLEX: u32 = 1 << 11;
pub const GEM_BASE_100_T2_FULL_DUPLEX: u32 = 1 << 10;
pub const GEM_BASE_100_T2_HALF_DUPLEX: u32 = 1 << 9;
pub const GEM_EXTENDED_STATUS: u32 = 1 << 8;
pub const GEM_AUTO_NEG_COMPLETE: u32 = 1 << 5;
pub const GEM_REMOTE_FAULT: u32 = 1 << 4;
pub const GEM_AUTO_NEG_ABILITY: u32 = 1 << 3;
pub const GEM_LINK_STATUS: u32 = 1 << 2;
pub const GEM_EXTENDED_CAPABILITIES: u32 = 1 << 0;

/* General MAC PCS PHY Top ID register bit definitions */
/* General MAC PCS PHY Bottom ID register bit definitions */

pub const GEM_ID_CODE: u32 = 0xFFFF;

/* General MAC PCS Autonegotiation Advertisment register bit definitions */

pub const GEM_AN_AV_NEXT_PAGE: u32 = 1 << 15;
pub const GEM_AN_AV_REMOTE_FAULT: u32 = (1 << 12) | (1 << 13);
pub const GEM_AN_AV_PAUSE: u32 = (1 << 7) | (1 << 8);
pub const GEM_AN_AV_HALF_DUPLEX: u32 = 1 << 6;
pub const GEM_AN_AV_FULL_DUPLEX: u32 = 1 << 5;

/* General MAC PCS Autonegotiation Link Partner Base register bit definitions */

pub const GEM_LINK_PARTNER_NEXT_PAGE_STATUS: u32 = 1 << 15;
pub const GEM_LINK_PARTNER_ACKNOWLEDGE: u32 = 1 << 14;
pub const GEM_LINK_PARTNER_REMOTE_FAULT_DUPLEX_MODE: u32 = (1 << 12) | (1 << 13);
pub const GEM_LINK_PARTNER_SPEED: u32 = (1 << 10) | (1 << 11);
pub const GEM_LINK_PARTNER_PAUSE: u32 = (1 << 7) | (1 << 8);
pub const GEM_LINK_PARTNER_HALF_DUPLEX: u32 = 1 << 6;
pub const GEM_LINK_PARTNER_FULL_DUPLEX: u32 = 1 << 5;

/* General MAC PCS Autonegotiation Next Page Ability register bit definitions */

pub const GEM_NEXT_PAGE_CAPABILITY: u32 = 1 << 2;
pub const GEM_PAGE_RECEIVED: u32 = 1 << 1;

/* General MAC PCS Autonegotiation Next Page Transmit register bit definitions */
/* General MAC PCS Autonegotiation Next Page Receive register bit definitions */

pub const GEM_NEXT_PAGE_TO_TRANSMIT: u32 = 1 << 15;
pub const GEM_NEXT_PAGE_TO_RECEIVE: u32 = 1 << 15;
pub const GEM_ACKNOWLEDGE: u32 = 1 << 14;
pub const GEM_MESSAGE_PAGE_INDICATOR: u32 = 1 << 13;
pub const GEM_ACKNOWLEDGE_2: u32 = 1 << 12;
pub const GEM_TOGGLE: u32 = 1 << 11;
pub const GEM_AN_MESSAGE: u32 = 0b11111111111;

/* General MAC PCS Autonegotiation Extended Status register bit definitions */

pub const GEM_FULL_DUPLEX_1000BASE_X: u32 = 1 << 15;
pub const GEM_HALF_DUPLEX_1000BASE_X: u32 = 1 << 14;
pub const GEM_FULL_DUPLEX_1000BASE_T: u32 = 1 << 13;
pub const GEM_HALF_DUPLEX_1000BASE_T: u32 = 1 << 12;

/* General MAC Received LPI Transitions register bit definitions */
/* General MAC Transmitted LPI Transitions register bit definitions */
/* eMAC Received LPI Transitions register bit definitions */
/* eMAC Transmitted LPI Transitions register bit definitions */

pub const GEM_LPI_COUNT: u32 = 0xFFFF;

/* General MAC Received LPI Time register bit definitions */
/* General MAC Transmitted LPI Time register bit definitions */
/* eMAC Received LPI Time register bit definitions */
/* eMAC Transmitted LPI Time register bit definitions */

pub const GEM_LPI_TIME: u32 = 0xFFFFFF;

/* General MAC Design Configuration Debug 1 register bit definitions */
/* eMAC Design Configuration Debug 1 register bit definitions */

pub const GEM_AXI_CACHE_VALUE: u32 = 0b1111 << 28;
pub const GEM_DMA_BUS_WIDTH: u32 = (1 << 25) | (1 << 26) | (1 << 27);
pub const GEM_EXCLUDE_CBS: u32 = 1 << 24;
pub const GEM_IRQ_READ_CLEAR: u32 = 1 << 23;
pub const GEM_NO_SNAPSHOT: u32 = 1 << 22;
pub const GEM_NO_STATS: u32 = 1 << 21;
pub const GEM_USER_IN_WIDTH: u32 = 0b11111 << 15;
pub const GEM_USER_OUT_WIDTH: u32 = 0b11111 << 10;
pub const GEM_USER_IO: u32 = 1 << 9;
pub const GEM_EXT_FIFO_INTERFACE: u32 = 1 << 6;
pub const GEM_INT_LOOPBACK: u32 = 1 << 4;
pub const GEM_EXCLUDE_QBV: u32 = 1 << 1;
pub const GEM_NO_PCS: u32 = 1 << 0;

/* General MAC Design Configuration Debug 2 register bit definitions */
/* eMAC Design Configuration Debug 2 register bit definitions */

pub const GEM_SPRAM: u32 = 1 << 31;
pub const GEM_AXI: u32 = 1 << 30;
pub const GEM_TX_PBUF_ADDR: u32 = 0xF << 26;
pub const GEM_RX_PBUF_ADDR: u32 = 0xF << 22;
pub const GEM_TX_PKT_BUFFER: u32 = 1 << 21;
pub const GEM_RX_PKT_BUFFER: u32 = 1 << 20;
pub const GEM_HPROT_VALUE: u32 = 0xF << 16;
pub const GEM_JUMBO_MAX_LENGTH: u32 = 0x3FFF;

/* General MAC Design Configuration Debug 3 register bit definitions */
/* eMAC Design Configuration Debug 3 register bit definitions */

pub const GEM_NUM_SPEC_ADD_FILTERS: u32 = 0x3F << 24;

/* General MAC Design Configuration Debug 5 register bit definitions */
/* eMAC Design Configuration Debug 5 register bit definitions */

pub const GEM_AXI_PROT_VALUE: u32 = (1 << 29) | (1 << 30) | (1 << 31);
pub const GEM_TSU_CLK: u32 = 1 << 28;
pub const GEM_RX_BUFFER_LENGTH_DEF: u32 = 0xFF << 20;
pub const GEM_TX_PBUF_SIZE_DEF: u32 = 1 << 19;
pub const GEM_RX_PBUF_SIZE_DEF: u32 = (1 << 17) | (1 << 18);
pub const GEM_ENDIAN_SWAP_DEF: u32 = (1 << 15) | (1 << 16);
pub const GEM_MDC_CLOCK_DIV: u32 = (1 << 12) | (1 << 13) | (1 << 14);
pub const GEM_DMA_BUS_WIDTH_DEF: u32 = (1 << 10) | (1 << 11);
pub const GEM_PHY_IDENT: u32 = 1 << 9;
pub const GEM_TSU: u32 = 1 << 8;
pub const GEM_TX_FIFO_CNT_WIDTH: u32 = 0xF << 4;
pub const GEM_RX_FIFO_CNT_WIDTH: u32 = 0xF;

/* General MAC Design Configuration Debug 6 register bit definitions */
/* eMAC Design Configuration Debug 6 register bit definitions */

pub const GEM_PBUF_LSO: u32 = 1 << 27;
pub const GEM_PBUF_RSC: u32 = 1 << 26;
pub const GEM_PBUF_CUTTHRU: u32 = 1 << 25;
pub const GEM_PFC_MULTI_QUANTUM: u32 = 1 << 24;
pub const GEM_DMA_ADDR_WIDTH_IS_64B: u32 = 1 << 23;
pub const GEM_HOST_IF_SOFT_SEL: u32 = 1 << 22;
pub const GEM_TX_ADD_FIFO_IF: u32 = 1 << 21;
pub const GEM_EXT_TSU_TIMER: u32 = 1 << 20;
pub const GEM_TX_PBUF_QUEUE_SEGMENT_SIZE: u32 = 0xF << 16;
pub const GEM_DMA_PRIORITY_QUEUE15: u32 = 1 << 15;
pub const GEM_DMA_PRIORITY_QUEUE14: u32 = 1 << 14;
pub const GEM_DMA_PRIORITY_QUEUE13: u32 = 1 << 13;
pub const GEM_DMA_PRIORITY_QUEUE12: u32 = 1 << 12;
pub const GEM_DMA_PRIORITY_QUEUE11: u32 = 1 << 11;
pub const GEM_DMA_PRIORITY_QUEUE10: u32 = 1 << 10;
pub const GEM_DMA_PRIORITY_QUEUE9: u32 = 1 << 9;
pub const GEM_DMA_PRIORITY_QUEUE8: u32 = 1 << 8;
pub const GEM_DMA_PRIORITY_QUEUE7: u32 = 1 << 7;
pub const GEM_DMA_PRIORITY_QUEUE6: u32 = 1 << 6;
pub const GEM_DMA_PRIORITY_QUEUE5: u32 = 1 << 5;
pub const GEM_DMA_PRIORITY_QUEUE4: u32 = 1 << 4;
pub const GEM_DMA_PRIORITY_QUEUE3: u32 = 1 << 3;
pub const GEM_DMA_PRIORITY_QUEUE2: u32 = 1 << 2;
pub const GEM_DMA_PRIORITY_QUEUE1: u32 = 1 << 1;

/* General MAC Design Configuration Debug 7 register bit definitions */
/* eMAC Design Configuration Debug 7 register bit definitions */

pub const GEM_TX_PBUF_NUM_SEGMENTS_Q7: u32 = 0xF << 28;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q6: u32 = 0xF << 24;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q5: u32 = 0xF << 20;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q4: u32 = 0xF << 16;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q3: u32 = 0xF << 12;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q2: u32 = 0xF << 8;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q1: u32 = 0xF << 4;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q0: u32 = 0xF;

/* General MAC Design Configuration Debug 8 register bit definitions */
/* eMAC Design Configuration Debug 8 register bit definitions */

pub const GEM_NUM_TYPE1_SCREENERS: u32 = 0x3F << 24;
pub const GEM_NUM_TYPE2_SCREENERS: u32 = 0x3F << 16;
pub const GEM_NUM_SCR2_ETHTYPE_REGS: u32 = 0x3F << 8;
pub const GEM_NUM_SCR2_COMPARE_REGS: u32 = 0xF;

/* General MAC Design Configuration Debug 9 register bit definitions */
/* eMAC Design Configuration Debug 9 register bit definitions */

pub const GEM_TX_PBUF_NUM_SEGMENTS_Q15: u32 = 0xF << 28;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q14: u32 = 0xF << 24;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q13: u32 = 0xF << 20;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q12: u32 = 0xF << 16;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q11: u32 = 0xF << 12;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q10: u32 = 0xF << 8;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q9: u32 = 0xF << 4;
pub const GEM_TX_PBUF_NUM_SEGMENTS_Q8: u32 = 0xF;

/* General MAC Design Configuration Debug 10 register bit definitions */
/* eMAC Design Configuration Debug 10 register bit definitions */

pub const GEM_EMAC_BUS_WIDTH: u32 = 0xF << 28;
pub const GEM_TX_PBUF_DATA: u32 = 0xF << 24;
pub const GEM_RX_PBUF_DATA: u32 = 0xF << 20;
pub const GEM_AXI_ACCESS_PIPELINE_BITS: u32 = 0xF << 16;
pub const GEM_AXI_TX_DESCR_RD_BUFF_BITS: u32 = 0xF << 12;
pub const GEM_AXI_RX_DESCR_RD_BUFF_BITS: u32 = 0xF << 8;
pub const GEM_AXI_TX_DESCR_WR_BUFF_BITS: u32 = 0xF << 4;
pub const GEM_AXI_RX_DESCR_WR_BUFF_BITS: u32 = 0xF;

/* General MAC Design Configuration Debug 11 register bit definitions */
/* eMAC Design Configuration Debug 11 register bit definitions */

pub const GEM_PROTECT_DESCR_ADDR: u32 = 1 << 4;
pub const GEM_PROTECT_TSU: u32 = 1 << 3;
pub const GEM_ADD_CSR_PARITY: u32 = 1 << 2;
pub const GEM_ADD_DP_PARITY: u32 = 1 << 1;
pub const GEM_ADD_ECC_DPRAM: u32 = 1 << 0;

/* General MAC Design Configuration Debug 12 register bit definitions */
/* eMAC Design Configuration Debug 12 register bit definitions */

pub const GEM_GEM_HAS_802P3_BR: u32 = 1 << 25;
pub const GEM_EMAC_TX_PBUF_ADDR: u32 = 0xF << 21;
pub const GEM_EMAC_RX_PBUF_ADDR: u32 = 0xF << 17;
pub const GEM_GEM_HAS_CB: u32 = 1 << 16;
pub const GEM_GEM_CB_HISTORY_LEN: u32 = 0xFF << 8;
pub const GEM_GEM_NUM_CB_STREAMS: u32 = 0xF;

/* General MAC Queue 1 DMA Receive Buffer Size register bit definitions */
/* General MAC Queue 2 DMA Receive Buffer Size register bit definitions */
/* General MAC Queue 3 DMA Receive Buffer Size register bit definitions */

pub const GEM_DMA_RX_Q_BUF_SIZE: u32 = 0xFF;

/* General MAC CBS Control register bit definitions */
/* eMAC CBS Control register bit definitions */

pub const GEM_CBS_ENABLE_QUEUE_B: u32 = 1 << 1;
pub const GEM_CBS_ENABLE_QUEUE_A: u32 = 1 << 0;

/* General MAC TX BD Control register bit definitions */
/* General MAC RX BD Control register bit definitions */
/* eMAC TX BD Control register bit definitions */
/* eMAC RX BD Control register bit definitions */

pub const GEM_BD_TS_MODE: u32 = (1 << 4) | (1 << 5);
pub const GEM_BD_TS_MODE_SHIFT: u32 = 4;

/* General MAC WD Counter register bit definitions */

pub const GEM_RX_BD_REREAD_TIMER: u32 = 0xFF;

/* General MAC AXI TX Full Threshold 0 register bit definitions */

pub const GEM_AXI_TX_FULL_ADJ_0: u32 = 0x3FFF << 16;
pub const GEM_AXI_TX_FULL_ADJ_1: u32 = 0xFFF;

/* General MAC AXI TX Full Threshold 1 register bit definitions */

pub const GEM_AXI_TX_FULL_ADJ_2: u32 = 0x3FFF << 16;
pub const GEM_AXI_TX_FULL_ADJ_3: u32 = 0xFFF;

/* General Screening Type 1 Register register bit definitions */

pub const GEM_DROP_ON_MATCH: u32 = 1 << 30;
pub const GEM_UDP_PORT_MATCH_ENABLE: u32 = 1 << 29;
pub const GEM_DSTC_ENABLE: u32 = 1 << 28;
pub const GEM_UDP_PORT_MATCH: u32 = 0xF << 12;
pub const GEM_DSTC_MATCH: u32 = 0xF << 4;
pub const GEM_QUEUE_NUMBER: u32 = 0xF;

pub const GEM_UDP_PORT_MATCH_SHIFT: u32 = 12;
pub const GEM_DSTC_MATCH_SHIFT: u32 = 4;

/* General Screening Type 2 Register register bit definitions */

pub const GEM_T2_DROP_ON_MATCH: u32 = 1 << 31;
pub const GEM_COMPARE_C_ENABLE: u32 = 1 << 30;
pub const GEM_COMPARE_C: u32 = 0xF << 25;
pub const GEM_COMPARE_B_ENABLE: u32 = 1 << 24;
pub const GEM_COMPARE_B: u32 = 0xF << 19;
pub const GEM_COMPARE_A_ENABLE: u32 = 1 << 18;
pub const GEM_COMPARE_A: u32 = 0xF << 13;
pub const GEM_ETHERTYPE_ENABLE: u32 = 1 << 12;
pub const GEM_ETHERTYPE_REG_INDEX: u32 = 0x3F << 9;
pub const GEM_VLAN_ENABLE: u32 = 1 << 8;
pub const GEM_VLAN_PRIORITY: u32 = 0xF << 4;

pub const GEM_COMPARE_C_SHIFT: u32 = 25;
pub const GEM_COMPARE_B_SHIFT: u32 = 19;
pub const GEM_COMPARE_A_SHIFT: u32 = 13;
pub const GEM_ETHERTYPE_REG_INDEX_SHIFT: u32 = 9;
pub const GEM_VLAN_PRIORITY_SHIFT: u32 = 4;

/* General MAC TX Schedule Control register bit definitions */

pub const GEM_TX_SCHED_Q3: u32 = 1 << 6 | 1 << 7;
pub const GEM_TX_SCHED_Q2: u32 = 1 << 4 | 1 << 5;
pub const GEM_TX_SCHED_Q1: u32 = 1 << 2 | 1 << 3;
pub const GEM_TX_SCHED_Q0: u32 = 1 << 0 | 1 << 1;

/* General MAC TX Bandwidth Rate Limit Queue 0 to 3 register bit definitions */

pub const GEM_DWRR_ETS_WEIGHT_Q3: u32 = 0xFF << 24;
pub const GEM_DWRR_ETS_WEIGHT_Q2: u32 = 0xFF << 16;
pub const GEM_DWRR_ETS_WEIGHT_Q1: u32 = 0xFF << 8;
pub const GEM_DWRR_ETS_WEIGHT_Q0: u32 = 0xFF;

/* General MAC TX Queue Segment Alloc Queue 0 to 3 register bit definitions */

pub const GEM_SEGMENT_ALLOC_Q3: u32 = 1 << 12 | 1 << 13 | 1 << 14;
pub const GEM_SEGMENT_ALLOC_Q2: u32 = 1 << 8 | 1 << 9 | 1 << 10;
pub const GEM_SEGMENT_ALLOC_Q1: u32 = 1 << 4 | 1 << 5 | 1 << 6;
pub const GEM_SEGMENT_ALLOC_Q0: u32 = 1 << 0 | 1 << 1 | 1 << 2;

/* General MAC Screening Type 2 Ethertype Reg 0 register bit definitions */

pub const GEM_COMPARE_VALUE: u32 = 0xFFFF;

/* General MAC Type 2 Compare 0 Word 0 register bit definitions */
/* eMAC Type 2 Compare 0 Word 0 register bit definitions */
/* eMAC Type 2 Compare 1 Word 0 register bit definitions */
/* eMAC Type 2 Compare 2 Word 0 register bit definitions */
/* eMAC Type 2 Compare 3 Word 0 register bit definitions */
/* eMAC Type 2 Compare 4 Word 0 register bit definitions */
/* eMAC Type 2 Compare 5 Word 0 register bit definitions */

pub const GEM_W0_COMPARE_VALUE: u32 = 0xFFFF << 16;
pub const GEM_W0_MASK_VALUE: u32 = 0xFFFF;

/* General MAC Type 2 Compare 0 Word 1 register bit definitions */
/* eMAC Type 2 Compare 0 Word 1 register bit definitions */
/* eMAC Type 2 Compare 1 Word 1 register bit definitions */
/* eMAC Type 2 Compare 2 Word 1 register bit definitions */
/* eMAC Type 2 Compare 3 Word 1 register bit definitions */
/* eMAC Type 2 Compare 4 Word 1 register bit definitions */
/* eMAC Type 2 Compare 5 Word 1 register bit definitions */

pub const GEM_COMPARE_VLAN_ID: u32 = 1 << 10;
pub const GEM_DISABLE_MASK: u32 = 1 << 9;
pub const GEM_COMPARE_OFFSET: u32 = 1 << 7 | 1 << 8;
pub const GEM_COMPARE_S_TAG: u32 = 1 << 7;
pub const GEM_OFFSET_VALUE: u32 = 0x7F;

pub const GEM_COMPARE_OFFSET_SHIFT: u32 = 7;

/* General MAC Enst Start Time Queue 0 register bit definitions */
/* General MAC Enst Start Time Queue 1 register bit definitions */
/* General MAC Enst Start Time Queue 2 register bit definitions */
/* General MAC Enst Start Time Queue 3 register bit definitions */
/* eMAC Enst Start Time register bit definitions */

pub const GEM_START_TIME_SEC: u32 = 1 << 30 | 1 << 31;
pub const GEM_START_TIME_NSEC: u32 = 0x3FFFFFFF;

/* General MAC Enst Start Time Queue 0 register bit definitions */
/* General MAC Enst Start Time Queue 1 register bit definitions */
/* General MAC Enst Start Time Queue 2 register bit definitions */
/* General MAC Enst Start Time Queue 3 register bit definitions */
/* General MAC Enst Off Time Queue 0 register bit definitions */
/* General MAC Enst Off Time Queue 1 register bit definitions */
/* General MAC Enst Off Time Queue 2 register bit definitions */
/* General MAC Enst Off Time Queue 3 register bit definitions */
/* eMAC Enst Start Time register bit definitions */
/* eMAC Enst Off Time register bit definitions */

pub const GEM_ON_OFF_TIME: u32 = 0x1FFFF;

/* General MAC Enst Control register bit definitions */
/* eMAC Enst Control register bit definitions */

pub const GEM_ENST_DISABLE_Q_3: u32 = 1 << 19;
pub const GEM_ENST_DISABLE_Q_2: u32 = 1 << 18;
pub const GEM_ENST_DISABLE_Q_1: u32 = 1 << 17;
pub const GEM_ENST_DISABLE_Q_0: u32 = 1 << 16;
pub const GEM_ENST_ENABLE_Q_3: u32 = 1 << 3;
pub const GEM_ENST_ENABLE_Q_2: u32 = 1 << 2;
pub const GEM_ENST_ENABLE_Q_1: u32 = 1 << 1;
pub const GEM_ENST_ENABLE_Q_0: u32 = 1 << 0;

/* General MAC MMSL Control register bit definitions */

pub const GEM_MMSL_DEBUG_MODE: u32 = 1 << 6;
pub const GEM_ROUTE_RX_TO_PMAC: u32 = 1 << 5;
pub const GEM_RESTART_VER: u32 = 1 << 4;
pub const GEM_PRE_ENABLE: u32 = 1 << 3;
pub const GEM_VERIFY_DISABLE: u32 = 1 << 2;
pub const GEM_ADD_FRAG_SIZE: u32 = 1 << 1 | 1 << 0;

/* General MAC MMSL Status register bit definitions */

pub const GEM_SMD_ERROR: u32 = 1 << 10;
pub const GEM_FRER_COUNT_ERR: u32 = 1 << 9;
pub const GEM_SMDC_ERROR: u32 = 1 << 8;
pub const GEM_SMDS_ERROR: u32 = 1 << 7;
pub const GEM_RCV_V_ERROR: u32 = 1 << 6;
pub const GEM_RCV_R_ERROR: u32 = 1 << 5;
pub const GEM_VERIFY_STATUS: u32 = 1 << 4 | 1 << 5 | 1 << 6;
pub const GEM_RESPOND_STATUS: u32 = 1 << 1;
pub const GEM_PRE_ACTIVE: u32 = 1 << 0;

pub const GEM_VERIFY_STATUS_SHIFT: u32 = 2;

pub const GEM_VERIFY_INIT: u32 = 0x00;
pub const GEM_VERIFY_IDLE: u32 = 0x01;
pub const GEM_VERIFY_SEND: u32 = 0x02;
pub const GEM_VERIFY_WAIT: u32 = 0x03;
pub const GEM_VERIFY_DONE_OK: u32 = 0x04;
pub const GEM_VERIFY_DONE_FAIL: u32 = 0x05;

/* General MAC MMSL Error Stats register bit definitions */

pub const GEM_SMD_ERR_COUNT: u32 = 0xFF << 16;
pub const GEM_ASS_ERR_COUNT: u32 = 0xFF;

pub const GEM_SMD_ERR_COUNT_SHIFT: u32 = 16;

/* General MAC MMSL Ass OK Count register bit definitions */

pub const GEM_ASS_OK_COUNT: u32 = 0x1FFFF;

/* General MAC MMSL Frag Count RX register bit definitions */
/* General MAC MMSL Frag Count TX register bit definitions */

pub const GEM_FRAG_COUNT: u32 = 0x1FFFF;

/* General MAC MMSL Interrupt Status register bit definitions */
/* General MAC MMSL Interrupt Enable register bit definitions */
/* General MAC MMSL Interrupt Disable register bit definitions */
/* General MAC MMSL Interrupt Mask register bit definitions */

pub const GEM_INT_SMD_ERROR: u32 = 1 << 5;
pub const GEM_INT_FRER_COUNT_ERR: u32 = 1 << 4;
pub const GEM_INT_SMDC_ERROR: u32 = 1 << 3;
pub const GEM_INT_SMDS_ERROR: u32 = 1 << 2;
pub const GEM_INT_RCV_V_ERROR: u32 = 1 << 1;
pub const GEM_INT_RCV_R_ERROR: u32 = 1 << 0;
