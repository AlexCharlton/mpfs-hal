#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

/// This example illustrates the memory layout of the MPFS.
/// See https://github.com/polarfire-soc/polarfire-soc-documentation/blob/master/knowledge-base/mpfs-memory-hierarchy.md
/// for details.

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    let flush_addr = 0x0201_0200 as *const u64;
    let ddr_cached_low_addr = 0x8000_0000;
    let ddr_uncached_low_addr = 0xC000_0000;
    let ddr_cached_high_addr = 0x10_0000_0000;
    let ddr_uncached_high_addr = 0x14_0000_0000;

    let addr1 = 0x0700_0000 as *const u8;
    // The cache block size is 64 bytes, so this address is in a different cache block.
    let addr2 = 0x0700_0080 as *const u8; // More than 64 bytes away from addr1

    println!("Value at 0x00_8700_0000: {:#x} (DDR Cached Low)", unsafe {
        *addr1.add(ddr_cached_low_addr)
    });
    println!("Value at 0x00_8700_0080: {:#x} (DDR Cached Low)", unsafe {
        *addr2.add(ddr_cached_low_addr)
    });
    println!(
        "Value at 0x00_C700_0000: {:#x} (DDR Uncached Low)",
        unsafe { *addr1.add(ddr_uncached_low_addr) }
    );
    println!(
        "Value at 0x00_C700_0080: {:#x} (DDR Uncached Low)",
        unsafe { *addr2.add(ddr_uncached_low_addr) }
    );
    println!("Value at 0x10_0700_0000: {:#x} (DDR Cached High)", unsafe {
        *addr1.add(ddr_cached_high_addr)
    });
    println!("Value at 0x10_0700_0080: {:#x} (DDR Cached High)", unsafe {
        *addr2.add(ddr_cached_high_addr)
    });
    println!(
        "Value at 0x14_0700_0000: {:#x} (DDR Uncached High)",
        unsafe { *addr1.add(ddr_uncached_high_addr) }
    );
    println!(
        "Value at 0x14_0700_0080: {:#x} (DDR Uncached High)\n",
        unsafe { *addr2.add(ddr_uncached_high_addr) }
    );

    unsafe {
        *(addr1.add(ddr_cached_low_addr) as *mut u8) = 0xAA;
        *(addr2.add(ddr_cached_low_addr) as *mut u8) = 0xAA;
        *(flush_addr as *mut u64) = addr1.add(ddr_cached_low_addr) as u64;
    };
    println!("Wrote 0xAA to 0x87000000");
    println!("Wrote 0xAA to 0x87000080");
    println!("Flushed 0x87000000 to DDR\n");

    println!(
        "Value at 0x00_8700_0000: {:#x} (Should be set: we just wrote to it)",
        unsafe { *addr1.add(ddr_cached_low_addr) }
    );
    println!(
        "Value at 0x00_8700_0080: {:#x} (Should be set: we just wrote to it)",
        unsafe { *addr2.add(ddr_cached_low_addr) }
    );
    println!(
        "Value at 0x00_C700_0000: {:#x} (Should be set: we flushed the corresponding cache block)",
        unsafe { *addr1.add(ddr_uncached_low_addr) }
    );
    println!(
        "Value at 0x00_C700_0080: {:#x} (Might not be set: we haven't flushed it yet)",
        unsafe { *addr2.add(ddr_uncached_low_addr) }
    );
    println!("Value at 0x10_0700_0000: {:#x} (Should be set)", unsafe {
        *addr1.add(ddr_cached_high_addr)
    });

    println!(
        "Value at 0x10_0700_0080: {:#x} (Might not be set)",
        unsafe { *addr2.add(ddr_cached_high_addr) }
    );
    println!("Value at 0x14_0700_0000: {:#x} (Should be set)", unsafe {
        *addr1.add(ddr_uncached_high_addr)
    });
    println!(
        "Value at 0x14_0700_0080: {:#x} (Might not be set)",
        unsafe { *addr2.add(ddr_uncached_high_addr) }
    );
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
