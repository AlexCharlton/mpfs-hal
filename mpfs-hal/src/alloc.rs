extern crate alloc;

use super::pac;
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub unsafe fn init_heap() {
    HEAP.init(
        pac::last_linked_address(),
        pac::last_address() - pac::last_linked_address(),
    )
}
