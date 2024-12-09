use super::bindings::*;

pub const GPIO0_LO: *mut GPIO_TypeDef = 0x20120000 as *mut GPIO_TypeDef;
pub const GPIO1_LO: *mut GPIO_TypeDef = 0x20121000 as *mut GPIO_TypeDef;
pub const GPIO2_LO: *mut GPIO_TypeDef = 0x20122000 as *mut GPIO_TypeDef;
pub const GPIO0_HI: *mut GPIO_TypeDef = 0x28120000 as *mut GPIO_TypeDef;
pub const GPIO1_HI: *mut GPIO_TypeDef = 0x28121000 as *mut GPIO_TypeDef;
pub const GPIO2_HI: *mut GPIO_TypeDef = 0x28122000 as *mut GPIO_TypeDef;
