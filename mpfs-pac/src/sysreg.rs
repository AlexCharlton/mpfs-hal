/// MSS System Register access
///
use super::bindings::*;

pub const SYSREG: *mut mss_sysreg_t = BASE32_ADDR_MSS_SYSREG as *mut mss_sysreg_t;
