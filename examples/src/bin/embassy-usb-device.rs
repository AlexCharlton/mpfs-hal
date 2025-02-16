#![no_std]
#![no_main]

use aligned::{Aligned, A4};
use mpfs_hal::pac;

#[macro_use]
extern crate mpfs_hal;

// String Descriptor Indexes
const USB_STRING_DESCRIPTOR_IDX_LANGID: u8 = 0x00;
const USB_STRING_DESCRIPTOR_IDX_MANUFACTURER: u8 = 0x01;
const USB_STRING_DESCRIPTOR_IDX_PRODUCT: u8 = 0x02;
const USB_STRING_DESCRIPTOR_IDX_SERIAL: u8 = 0x03;
const USB_STRING_DESCRIPTOR_IDX_CONFIG: u8 = 0x04;
const USB_STRING_DESCRIPTOR_IDX_INTERFACE: u8 = 0x05;

// String constants
const USB_STRING_MANUFACTURER: &str = "ACME, Inc.";
const USB_STRING_PRODUCT: &str = "mpfs-hal Mouse";
const USB_STRING_SERIAL: &str = "HID1234";
const USB_STRING_CONFIG: &str = "CFG0";
const USB_STRING_INTERFACE: &str = "Interface0";

const USB_MAX_STRING_DESCRIPTOR_SIZE: usize = 64;

static mut G_STRING_DESCRIPTOR: Aligned<A4, [u8; USB_MAX_STRING_DESCRIPTOR_SIZE]> =
    Aligned([0; USB_MAX_STRING_DESCRIPTOR_SIZE]);

// This needs to be 4-byte aligned for DMA
static mut REPORT: Aligned<A4, pac::mss_usbd_hid_report_t> = Aligned(pac::mss_usbd_hid_report_t {
    buttons: 0,
    x_move: 0,
    y_move: 0,
    wheel: 0,
});

// Device descriptor
static DEVICE_DESCRIPTOR: Aligned<A4, [u8; pac::USB_STD_DEVICE_DESCR_LEN as usize]> = Aligned([
    pac::USB_STD_DEVICE_DESCR_LEN as u8,    // bLength
    pac::USB_DEVICE_DESCRIPTOR_TYPE as u8,  // bDescriptorType
    0x00,                                   // bcdUSB LSB
    0x02,                                   // bcdUSB MSB
    0x00,                                   // bDeviceClass
    0x00,                                   // bDeviceSubClass
    0x00,                                   // bDeviceProtocol
    0x40,                                   // bMaxPacketSize0
    0x14,                                   // idVendor LSB
    0x15,                                   // idVendor MSB
    0x01,                                   // idProduct LSB
    0x00,                                   // idProduct MSB
    0x00,                                   // bcdDevice LSB
    0x30,                                   // bcdDevice MSB
    USB_STRING_DESCRIPTOR_IDX_MANUFACTURER, // iManufacturer
    USB_STRING_DESCRIPTOR_IDX_PRODUCT,      // iProduct
    USB_STRING_DESCRIPTOR_IDX_SERIAL,       // iSerialNumber
    0x01,                                   // bNumConfigurations
]);

// Device qualifiers
static HS_DEV_QUALIFIER_DESCRIPTOR: Aligned<A4, [u8; pac::USB_STD_DEV_QUAL_DESCR_LENGTH as usize]> =
    Aligned([
        pac::USB_STD_DEV_QUAL_DESCR_LENGTH as u8,        // bLength
        pac::USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE as u8, // bDescriptorType
        0x00,                                            // bcdUSB LSB
        0x02,                                            // bcdUSB MSB
        0x00,                                            // bDeviceClass
        0x00,                                            // bDeviceSubClass
        0x00,                                            // bDeviceProtocol
        0x40,                                            // bMaxPacketSize0
        0x01,                                            // bNumConfigurations
        0x00,                                            // Reserved
    ]);

static FS_DEV_QUALIFIER_DESCRIPTOR: Aligned<A4, [u8; pac::USB_STD_DEV_QUAL_DESCR_LENGTH as usize]> =
    Aligned([
        pac::USB_STD_DEV_QUAL_DESCR_LENGTH as u8,        // bLength
        pac::USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE as u8, // bDescriptorType
        0x00,                                            // bcdUSB LSB
        0x02,                                            // bcdUSB MSB
        0x00,                                            // bDeviceClass
        0x00,                                            // bDeviceSubClass
        0x00,                                            // bDeviceProtocol
        0x40,                                            // bMaxPacketSize0
        0x01,                                            // bNumConfigurations
        0x00,                                            // Reserved
    ]);

static LANG_STRING_DESCRIPTOR: Aligned<A4, [u8; 4]> = Aligned([
    0x04,                                  // bLength
    pac::USB_STRING_DESCRIPTOR_TYPE as u8, // bDescriptorType
    0x09,                                  // LangID-LSB
    0x04,                                  // LangID-MSB
]);

unsafe extern "C" fn hid_device_descriptor(length: *mut u32) -> *mut u8 {
    *length = DEVICE_DESCRIPTOR.len() as u32;
    DEVICE_DESCRIPTOR.as_ptr() as *mut u8
}

unsafe extern "C" fn hid_device_qual_descriptor(
    speed: pac::mss_usb_device_speed_t,
    length: *mut u32,
) -> *mut u8 {
    let descriptor = match speed {
        pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS => &FS_DEV_QUALIFIER_DESCRIPTOR,
        _ => &HS_DEV_QUALIFIER_DESCRIPTOR,
    };
    *length = descriptor.len() as u32;
    descriptor.as_ptr() as *mut u8
}

fn hid_get_string(string: &str, dest: &mut Aligned<A4, [u8; 64]>) -> u8 {
    let mut idx = 0;
    for &byte in string.as_bytes() {
        dest[idx + 2] = byte;
        dest[idx + 3] = 0x00;
        idx += 2;
    }

    let length = (string.len() * 2 + 2) as u8;
    dest[0] = length;
    dest[1] = pac::USB_STRING_DESCRIPTOR_TYPE as u8;

    length
}

unsafe extern "C" fn hid_string_descriptor(index: u8, length: *mut u32) -> *mut u8 {
    match index {
        USB_STRING_DESCRIPTOR_IDX_LANGID => {
            *length = LANG_STRING_DESCRIPTOR.len() as u32;
            LANG_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        #[allow(static_mut_refs)]
        USB_STRING_DESCRIPTOR_IDX_MANUFACTURER => {
            *length = hid_get_string(USB_STRING_MANUFACTURER, &mut G_STRING_DESCRIPTOR) as u32;
            G_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        #[allow(static_mut_refs)]
        USB_STRING_DESCRIPTOR_IDX_PRODUCT => {
            *length = hid_get_string(USB_STRING_PRODUCT, &mut G_STRING_DESCRIPTOR) as u32;
            G_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        #[allow(static_mut_refs)]
        USB_STRING_DESCRIPTOR_IDX_SERIAL => {
            *length = hid_get_string(USB_STRING_SERIAL, &mut G_STRING_DESCRIPTOR) as u32;
            G_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        #[allow(static_mut_refs)]
        USB_STRING_DESCRIPTOR_IDX_CONFIG => {
            *length = hid_get_string(USB_STRING_CONFIG, &mut G_STRING_DESCRIPTOR) as u32;
            G_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        #[allow(static_mut_refs)]
        USB_STRING_DESCRIPTOR_IDX_INTERFACE => {
            *length = hid_get_string(USB_STRING_INTERFACE, &mut G_STRING_DESCRIPTOR) as u32;
            G_STRING_DESCRIPTOR.as_ptr() as *mut u8
        }
        _ => {
            *length = 0;
            core::ptr::null_mut()
        }
    }
}

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello, world!\nWe're going to wiggle your mouse along the X axis 🖱 ️↔️ ");
    loop {
        unsafe {
            if pac::MSS_USBD_HID_tx_done() == 1 {
                #[allow(static_mut_refs)]
                pac::MSS_USBD_HID_tx_report(
                    &REPORT as *const _ as *mut u8,
                    core::mem::size_of::<pac::mss_usbd_hid_report_t>() as u32,
                );
            } else {
                // Delay
                for _ in 0..50000 {
                    core::hint::spin_loop();
                }

                if REPORT.x_move < 30 {
                    REPORT.x_move += 1;
                } else {
                    REPORT.x_move = -30;
                }
            }
        }
    }
}

static DESCRIPTORS_CB: pac::mss_usbd_user_descr_cb_t = pac::mss_usbd_user_descr_cb_t {
    usbd_device_descriptor: Some(hid_device_descriptor),
    usbd_device_qual_descriptor: Some(hid_device_qual_descriptor),
    usbd_string_descriptor: Some(hid_string_descriptor),
};

fn init_usb() {
    println!("Initializing USB");
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_USB,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
        );
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_USB,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::init_usb_dma_upper_address();
        pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_USB_DMA_INT_OFFSET, 2);
        pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_USB_MC_INT_OFFSET, 2);

        pac::PLIC_EnableIRQ(pac::PLIC_IRQn_Type_PLIC_USB_DMA_INT_OFFSET);
        pac::PLIC_EnableIRQ(pac::PLIC_IRQn_Type_PLIC_USB_MC_INT_OFFSET);

        // Set up descriptor callbacks
        pac::MSS_USBD_set_descr_cb_handler(&DESCRIPTORS_CB as *const _ as *mut _);

        // Initialize HID Class driver
        pac::MSS_USBD_HID_init(pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS);

        // Initialize USB Device Core driver
        pac::MSS_USBD_init(pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS);
    }
    println!("USB initialized");
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
    init_usb();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
