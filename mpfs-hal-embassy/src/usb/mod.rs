pub(crate) mod common;

#[cfg(feature = "usb-device")]
pub mod device;

#[cfg(feature = "usb-host")]
pub mod host;

#[cfg(not(feature = "usb-host"))]
#[no_mangle]
#[doc(hidden)]
#[allow(non_upper_case_globals)]
pub static g_mss_usbh_cb: mpfs_hal::pac::mss_usbh_cb_t = mpfs_hal::pac::mss_usbh_cb_t {
    usbh_tx_complete: None,
    usbh_rx: None,
    usbh_cep: None,
    usbh_sof: None,
    usbh_connect: None,
    usbh_disconnect: None,
    usbh_vbus_error: None,
    usbh_babble_error: None,
    usbh_session_request: None,
    usbh_dma_handler: None,
};

#[cfg(not(feature = "usb-device"))]
#[no_mangle]
#[doc(hidden)]
#[allow(non_upper_case_globals)]
pub static g_mss_usbd_cb: mpfs_hal::pac::mss_usbd_cb_t = mpfs_hal::pac::mss_usbd_cb_t {
    usbd_ep_rx: None,
    usbd_ep_tx_complete: None,
    usbd_cep_setup: None,
    usbd_reset: None,
    usbd_suspend: None,
    usbd_resume: None,
    usbd_disconnect: None,
    usbd_dma_handler: None,
    usbd_sof: None,
    usbd_cep_rx: None,
    usbd_cep_tx_complete: None,
};
