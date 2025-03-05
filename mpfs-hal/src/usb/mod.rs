pub mod device;

#[cfg(feature = "usb-host")]
pub mod host;

#[cfg(not(feature = "usb-host"))]
#[no_mangle]
#[doc(hidden)]
#[allow(non_upper_case_globals)]
pub static g_mss_usbh_cb: crate::pac::mss_usbh_cb_t = crate::pac::mss_usbh_cb_t {
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
