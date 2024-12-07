// UART constants that bindgen can't generate

/***************************************************************************/
/**
 Data Bits Length
 ================
 The following defines are used to build the value of the MSS_UART_init(),
 MSS_UART_lin_init(), MSS_UART_irda_init(), and MSS_UART_smartcard_init()
 functions line_config parameter.

 | Constant             | Description                |
 |----------------------|----------------------------|
 | MSS_UART_DATA_5_BITS | 5 bits of data transmitted |
 | MSS_UART_DATA_6_BITS | 6 bits of data transmitted |
 | MSS_UART_DATA_7_BITS | 7 bits of data transmitted |
 | MSS_UART_DATA_8_BITS | 8 bits of data transmitted |

*/
pub const MSS_UART_DATA_5_BITS: u8 = 0x00;
pub const MSS_UART_DATA_6_BITS: u8 = 0x01;
pub const MSS_UART_DATA_7_BITS: u8 = 0x02;
pub const MSS_UART_DATA_8_BITS: u8 = 0x03;

/***************************************************************************/
/**
 Parity
 ======
 The following defines are used to build the value of the MSS_UART_init(),
 MSS_UART_lin_init(), MSS_UART_irda_init(), and MSS_UART_smartcard_init()
 functions line_config parameter.

 | Constant                | Description              |
 |-------------------------|--------------------------|
 | MSS_UART_NO_PARITY      | No parity                |
 | MSS_UART_ODD_PARITY     | Odd Parity               |
 | MSS_UART_EVEN_PARITY    | Even parity              |
 | MSS_UART_STICK_PARITY_0 | Stick parity bit to zero |
 | MSS_UART_STICK_PARITY_1 | Stick parity bit to one  |

*/
pub const MSS_UART_NO_PARITY: u8 = 0x00;
pub const MSS_UART_ODD_PARITY: u8 = 0x08;
pub const MSS_UART_EVEN_PARITY: u8 = 0x18;
pub const MSS_UART_STICK_PARITY_0: u8 = 0x38;
pub const MSS_UART_STICK_PARITY_1: u8 = 0x28;

/***************************************************************************/
/**
 Number of Stop Bits
 ===================
 The following defines are used to build the value of the MSS_UART_init(),
 MSS_UART_lin_init(), MSS_UART_irda_init(), and MSS_UART_smartcard_init()
 functions line_config parameter.

 | Constant                  | Description              |
 |---------------------------|--------------------------|
 | MSS_UART_ONE_STOP_BIT     | One Stop bit             |
 | MSS_UART_ONEHALF_STOP_BIT | One and a half Stop bits |
 | MSS_UART_TWO_STOP_BITS    | Two Stop bits            |

*/
pub const MSS_UART_ONE_STOP_BIT: u8 = 0x00;
pub const MSS_UART_ONEHALF_STOP_BIT: u8 = 0x04;
pub const MSS_UART_TWO_STOP_BITS: u8 = 0x04;

/***************************************************************************/
/**
 Receiver Error Status
 =====================
 The following defines are used to determine the UART receiver error type.
 These bit mask constants are used with the return value of the
 MSS_UART_get_rx_status() function to find out if any errors occurred while
 receiving data.


 | Constant               | Description                                |
 |------------------------|--------------------------------------------|
 | MSS_UART_NO_ERROR      | No error bit mask (0x00)                   |
 | MSS_UART_OVERUN_ERROR  | Overrun error bit mask (0x02)              |
 | MSS_UART_PARITY_ERROR  | Parity error bit mask (0x04)               |
 | MSS_UART_FRAMING_ERROR | Framing error bit mask (0x08)              |
 | MSS_UART_BREAK_ERROR   | Break error bit mask (0x10)                |
 | MSS_UART_FIFO_ERROR    | FIFO error bit mask (0x80)                 |
 | MSS_UART_INVALID_PARAM | Invalid function parameter bit mask (0xFF) |

*/
pub const MSS_UART_INVALID_PARAM: u8 = 0xFF;
pub const MSS_UART_NO_ERROR: u8 = 0x00;
pub const MSS_UART_OVERUN_ERROR: u8 = 0x02;
pub const MSS_UART_PARITY_ERROR: u8 = 0x04;
pub const MSS_UART_FRAMING_ERROR: u8 = 0x08;
pub const MSS_UART_BREAK_ERROR: u8 = 0x10;
pub const MSS_UART_FIFO_ERROR: u8 = 0x80;

/***************************************************************************/
/**
 Transmitter Status
 ==================
 The following definitions are used to determine the UART transmitter status.
 These bit mask constants are used with the return value of the
 MSS_UART_get_tx_status() function to find out the status of the transmitter.

 | Constant         | Description                                        |
 |------------------|----------------------------------------------------|
 | MSS_UART_TX_BUSY | Transmitter busy (0x00)                            |
 | MSS_UART_THRE    | Transmitter holding register empty bit mask (0x20) |
 | MSS_UART_TEMT    | Transmitter empty bit mask (0x40)                  |

*/
pub const MSS_UART_TX_BUSY: u8 = 0x00;
pub const MSS_UART_THRE: u8 = 0x20;
pub const MSS_UART_TEMT: u8 = 0x40;

/***************************************************************************/
/**
 Modem Status
 ============
 The following defines are used to determine the modem status. These bit
 mask constants are used with the return value of the
 MSS_UART_get_modem_status() function to find out the modem status of
 the UART.

 | Constant      | Description                                     |
 |---------------|-------------------------------------------------|
 | MSS_UART_DCTS | Delta clear to send bit mask (0x01)             |
 | MSS_UART_DDSR | Delta data set ready bit mask (0x02)            |
 | MSS_UART_TERI | Trailing edge of ring indicator bit mask (0x04) |
 | MSS_UART_DDCD | Delta data carrier detect bit mask (0x08)       |
 | MSS_UART_CTS  | Clear to send bit mask (0x10)                   |
 | MSS_UART_DSR  | Data set ready bit mask (0x20)                  |
 | MSS_UART_RI   | Ring indicator bit mask (0x40)                  |
 | MSS_UART_DCD  | Data carrier detect bit mask (0x80)             |

*/
pub const MSS_UART_DCTS: u8 = 0x01;
pub const MSS_UART_DDSR: u8 = 0x02;
pub const MSS_UART_TERI: u8 = 0x04;
pub const MSS_UART_DDCD: u8 = 0x08;
pub const MSS_UART_CTS: u8 = 0x10;
pub const MSS_UART_DSR: u8 = 0x20;
pub const MSS_UART_RI: u8 = 0x40;
pub const MSS_UART_DCD: u8 = 0x80;
