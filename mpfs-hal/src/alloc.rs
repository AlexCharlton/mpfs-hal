extern crate alloc;

use super::pac;
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub unsafe fn init_heap() {
    HEAP.init(pac::heap_start(), pac::heap_end() - pac::heap_start())
}
