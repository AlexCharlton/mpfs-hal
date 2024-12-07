use super::bindings::*;
use super::hart_id;

pub const CLINT: *mut CLINT_Type = CLINT_BASE as *mut CLINT_Type;

/// Raises a synchronous software interrupt by writing into the MSIP register.
///
/// # Safety
///
/// - Caller must ensure global interrupts are enabled
/// - `set_csr(mie, MIP_MSIP)` must be set on the hart receiving the interrupt
pub unsafe fn raise_soft_interrupt(hart_id: usize) {
    (*CLINT).MSIP[hart_id] = 0x1;
    riscv::asm::fence(); // equivalent to mb() memory barrier
}

/// Clears a synchronous software interrupt by clearing the MSIP register.
///
/// # Safety
///
/// This function reads a CSR register and modifies hardware state.
pub unsafe fn clear_soft_interrupt() {
    let hart_id = hart_id();
    (*CLINT).MSIP[hart_id] = 0x0;

    // Read back to ensure write completion
    let _ = (*CLINT).MSIP[hart_id];
}
