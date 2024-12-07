use super::bindings;

pub const MIP_SSIP: usize = 1 << bindings::IRQ_S_SOFT;
pub const MIP_HSIP: usize = 1 << bindings::IRQ_H_SOFT;
pub const MIP_MSIP: usize = 1 << bindings::IRQ_M_SOFT;
pub const MIP_STIP: usize = 1 << bindings::IRQ_S_TIMER;
pub const MIP_HTIP: usize = 1 << bindings::IRQ_H_TIMER;
pub const MIP_MTIP: usize = 1 << bindings::IRQ_M_TIMER;
pub const MIP_SEIP: usize = 1 << bindings::IRQ_S_EXT;
pub const MIP_HEIP: usize = 1 << bindings::IRQ_H_EXT;
pub const MIP_MEIP: usize = 1 << bindings::IRQ_M_EXT;
