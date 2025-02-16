use super::bindings::*;

pub const USB: *mut MSS_USB_TypeDef = USB_BASE as *mut MSS_USB_TypeDef;

const MSS_USB_ADDR_UPPER_OFFSET: u32 = 0x3FC;

pub fn set_usb_dma_upper_address(upper_address: u8) {
    unsafe {
        *((USB_BASE + MSS_USB_ADDR_UPPER_OFFSET) as *mut u8) = upper_address;
    }
}

/// Set the upper address bits of the address of the USB DMA
pub fn init_usb_dma_upper_address() {
    if cfg!(feature = "upper-memory-layout") {
        set_usb_dma_upper_address(0x10);
    } else {
        set_usb_dma_upper_address(0x00);
    }
}
