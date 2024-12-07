extern crate alloc;

use embedded_alloc::LlffHeap as Heap;
use mpfs_pac as sys;

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub unsafe fn init_heap() {
    HEAP.init(
        sys::last_linked_address(),
        sys::last_address() - sys::last_linked_address(),
    )
}
