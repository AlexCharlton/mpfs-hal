use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use static_cell::StaticCell;

use mpfs_hal::{qspi, Peripheral};

static QSPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, qspi::Qspi>> = StaticCell::new();

pub fn qspi_bus() -> &'static mut Mutex<CriticalSectionRawMutex, qspi::Qspi> {
    qspi::init();
    let qspi = qspi::Qspi::take().unwrap();
    QSPI_BUS.init(Mutex::new(qspi))
}
