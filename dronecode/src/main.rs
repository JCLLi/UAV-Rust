#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(log_syntax)]

extern crate alloc;

use crate::control::control_loop;
use alloc::format;
use log_storage_manager::LogStorageManager;
use core::alloc::Layout;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use tudelft_quadrupel::initialize::initialize;
use tudelft_quadrupel::led::Led::{Green, Red};
use tudelft_quadrupel::time::assembly_delay;
use tudelft_quadrupel::uart::send_bytes;
use tudelft_quadrupel::{entry, uart};
use fixed::types::I18F14;

mod working_mode;
mod control;
mod yaw_pitch_roll;
mod drone;
mod drone_transmission;
mod log_storage_manager;
mod controllers;

// Fixed point constants
const MOTOR_MIN: I18F14 = I18F14::lit("200");
const ZERO_POINT: I18F14 = I18F14::lit("32767");
const ZERO_POINT_YAW: I18F14 = I18F14::lit("8520");
const RESOLUTION: I18F14 = I18F14::lit("3.051850948e-5"); // 1/32767
const ANGLE_RESOLUTION: I18F14 = I18F14::lit("1.597945402e-5"); // 0.52359877/32767
const LIFT_RESOLUTION: I18F14 = I18F14::lit("1.52590219e-5"); // 1/65535
const MOTOR_RESOLUTION_CONTROL: I18F14 = I18F14::lit("1.66666666666666666e-3"); // 1/600
const MOTOR_RESOLUTION_MANUAL: I18F14 = I18F14::lit("2.5e-3"); // 1/400


/// The heap size of your drone code in bytes.
/// Note: there are 8192 bytes of RAM available.
const HEAP_SIZE: usize = 4096;

#[entry]
fn main() -> ! {
    {
        static mut HEAP_MEMORY: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

        // SAFETY: HEAP_MEMORY is a local static. That means, that at
        // the end of the surrounding block scope (not the main function, the local scope)
        // it is impossible to use HEAP_MEMORY *except* through this mutable reference
        // created here. Since that means that there's only one reference to HEAP_MEMORY,
        // this is safe.
        //
        // As soon as the first driver (led driver) is initialized, the yellow led turns on.
        // That's also the last thing that's turned off. If the yellow led stays on and your
        // program doesn't run, you know that the boot procedure has failed.
        
        initialize(unsafe { &mut HEAP_MEMORY }, false);
    }

    let storage = LogStorageManager::new(0x1FFF);
    storage.retrieve_loggings(0x1FFF);

    control_loop();
}

#[inline(never)]
// #[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // On panic:
    // * try and write the panic message on UART
    // * blink the red light

    if uart::is_initialized() {
        let msg = format!("{info}\n");
        send_bytes(msg.as_bytes());
    }

    // Start blinking red
    loop {
        let _ = Red.toggle();
        assembly_delay(1_000_000)
    }
}

#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    // When an allocation error happens, we panic.
    // However, we do not want to have UART initialized, since
    // the panic handler uses the allocator to print a message over
    // UART. However, if UART is not initialized, it won't attempt
    // to allocate the message.
    //
    // instead, to signal this, we turn the green light on too
    // (together with blinking red of the panic)
    Green.on();

    // Safety: after this we panic and go into an infinite loop
    unsafe { uart::uninitialize() };

    panic!("out of memory: {layout:?}");
}
