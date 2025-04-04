#![no_std]
#![no_main]

// https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_midi.rs used as reference

use aligned::{Aligned, A4};
use embassy_futures::join::join;
use embassy_usb::class::midi::MidiClass;
use embassy_usb::driver::EndpointError;
use midi_msg::{ChannelVoiceMsg, MidiMsg};

use mpfs_hal::usb::device::UsbDriver;
use mpfs_hal::Peripheral;

#[macro_use]
extern crate mpfs_hal;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let mut driver = UsbDriver::take().unwrap();
    driver.set_full_speed();
    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-MIDI example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors, and they should be 4-byte aligned.
    let mut config_descriptor = Aligned::<A4, _>([0; 256]);
    let mut bos_descriptor = Aligned::<A4, _>([0; 256]);
    let mut control_buf = Aligned::<A4, _>([0; 64]);

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        config_descriptor.as_mut_slice(),
        bos_descriptor.as_mut_slice(),
        &mut [], // no msos descriptors
        control_buf.as_mut_slice(),
    );

    // Create classes on the builder.
    let mut class = MidiClass::new(&mut builder, 1, 1, 64);
    // Build the builder.
    let mut usb = builder.build();
    // Run the USB device.
    let usb_fut = usb.run();

    let mut leds = mpfs_hal::gpio::Leds::take().unwrap();

    // Use the Midi class!
    let midi_fut = async {
        loop {
            class.wait_connection().await;
            log::info!("Connected");
            let _ = midi_display(&mut class, &mut leds).await;
            log::info!("Disconnected");
        }
    };

    println!("We're going to display incoming MIDI notes on the LEDs 🎶");

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, midi_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn midi_display<'d, 'a>(
    class: &'a mut MidiClass<'d, UsbDriver<'d>>,
    leds: &'a mut mpfs_hal::gpio::Leds,
) -> Result<(), Disconnected> {
    let mut buf = Aligned::<A4, _>([0; 64]);
    loop {
        let n = class.read_packet(&mut buf[..]).await?;
        let data = &buf[..n];
        log::info!("data: {:x?}", data);
        if data.len() > 0 {
            // Note on or note off
            if data[0] == 0x8 || data[0] == 0x9 {
                let msg = MidiMsg::from_midi(&data[1..]);
                log::info!("msg: {:?}", msg);
                if let Ok((
                    MidiMsg::ChannelVoice {
                        channel: _,
                        msg: ChannelVoiceMsg::NoteOn { note, .. },
                    },
                    _,
                )) = msg
                {
                    let led_num = note % 12;
                    leds.set_led(led_num as usize, true);
                }
                if let Ok((
                    MidiMsg::ChannelVoice {
                        channel: _,
                        msg: ChannelVoiceMsg::NoteOff { note, .. },
                    },
                    _,
                )) = msg
                {
                    let led_num = note % 12;
                    leds.set_led(led_num as usize, false);
                }
            }

            // Echo back the data
            class.write_packet(data).await?;
        }
    }
}

#[mpfs_hal::init_once]
fn config() {
    // We can expect to miss messages if we log at Info level
    mpfs_hal::init_logger(log::LevelFilter::Warn);
    mpfs_hal::gpio::init();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
