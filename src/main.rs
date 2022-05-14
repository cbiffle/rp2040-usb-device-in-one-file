// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Rust RP2040 USB Device Example.
//!
//! This is a working example of how to implement a USB device on RP2040 in
//! Rust, using _no_ external abstractions or indirections, and using the
//! simplest code flow I could devise.
//!
//! - No HALs.
//! - No callbacks.
//! - No interrupts.
//! - No SDK setup code preparing clocks or configuring peripherals behind your
//!   back.
//! - Every single thing you need to do to make an RP2040 speak USB, visible
//!   right here, in this very file.
//!
//! This is not the fastest, most functional, cleanest, or most elegant USB
//! device implementation. It's intended for reference and study. In particular,
//! the bulk of this file is a rather long single function, `main`. This was a
//! deliberate choice -- it makes it easier to follow the linear setup process
//! when it's written linearly, in my opinion. You can just read it from top to
//! bottom, like you're reading this text. Chopping it up into smaller functions
//! would likely make it easier to maintain and port, but would not (in my
//! opinion) make it easier to _understand._
//!
//! I wrote this out of frustration with existing USB examples, which either
//! expected me to read and understand codebases like TinyUSB, or sent me
//! climbing through layer after layer of abstractions that weren't relevant to
//! what I was trying to learn.
//!
//! This implementation, by contrast, should be readable using only this file
//! and the RP2040 datasheet. (And a handful of common Rust crates that are
//! being used for convenience, and have nothing to do with the hardware setup
//! or USB.)
//!
//! This code began life as a manual translation from the [pico-sdk C lowlevel
//! device example][lowlevel]. It's still compatible with that code, so you can
//! use their Python demo script to test the result. The functionality is
//! identical though the code is now fairly different as I aggressively
//! simplified and Rustified things.
//!
//! [lowlevel]: https://github.com/raspberrypi/pico-examples/blob/master/usb/device/dev_lowlevel/dev_lowlevel.c
//!
//! This was originally developed against Rust 1.59, stable.
//!
//! # A note on `unsafe`
//!
//! There's some `unsafe` here. It falls into two categories:
//!
//! - Like, actually `unsafe` -- the USB SRAM setup code and allocator, and the
//!   code that reconstructs references into USB SRAM when buffer activity
//!   happens. This is legit unsafe and I've given it explanatory comments.
//!
//! - `unsafe` because the `rp2040_pac` crate has marked a lot of API as being
//!   unsafe when it isn't. This is the vast majority of the `unsafe` and it
//!   could be removed by pushing patches upstream. In these cases I've omitted
//!   explanatory comments justifying the `unsafe`, because in each case, the
//!   justification is "idk why `rp2040_pac` makes me do this."
//!
//! You can recognize the second category of `unsafe` when you find it inside a
//! register `write` or `modify` operation.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use core::mem::MaybeUninit;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use zerocopy::{AsBytes, U16, FromBytes, Unaligned};
use byteorder::LittleEndian;
use num_traits::FromPrimitive;
use num_derive::FromPrimitive;

use panic_halt as _;

// GPIO numbers of pins used here, other than USB. The firmware toggles these
// pins to indicate activity, so that you can watch them on a logic analyzer to
// understand the event flow.
cfg_if::cfg_if! {
    if #[cfg(feature = "target-pico")] {
        const LED_PIN: u8 = 25; // any activity
        const SETUP_PIN: u8 = 0; // SETUP request being handled
        const BUFF_PIN: u8 = 1; // activity on a buffer
        const RESET_PIN: u8 = 2; // bus reset!
        const EP_PIN: [u8;3] = [3, 4, 5]; // activity on EP0, 1, 2
    } else if #[cfg(feature = "target-feather")] {
        const LED_PIN: u8 = 13; // any activity
        const SETUP_PIN: u8 = 0; // SETUP request being handled
        const BUFF_PIN: u8 = 1; // activity on a buffer
        const RESET_PIN: u8 = 2; // bus reset!
        const EP_PIN: [u8;3] = [3, 4, 5]; // activity on EP0, 1, 2
    } else {
        compile_error!("missing or unknown target-* feature");
    }
}

// Note: USB type definitions and support functions are _after_ main. I've tried
// to name them clearly, so that you should be able to read main without fully
// reading all the types and whatnot. However, if you would like to read things
// in C order (bottom up), you'll want to start farther down.

#[entry]
fn main() -> ! {
    // The chip has come out of reset, executed the boot ROM, and jumped into
    // our program. We are running at 6-ish MHz from the highly imprecise
    // internal ring oscillator ROSC, with all pins tristated.

    // Wake up the Peripheral Access Crate. This performs _no_ register writes
    // or setup, just sets a flag in RAM and gives us `p`.
    let p = rp2040_pac::Peripherals::take().unwrap();

    //////////////////////////////////////////////////////////////////////////
    // I/O pin configuration. Activate our diagnostic GPIOs so we can use them
    // for debugging etc.
    //
    // When in doubt, insert a `raise_pin(&p.SIO, LED_PIN);` somewhere and see
    // if the light comes on, to figure out if code makes it to that point.
    //
    // Note: the p.SIO peripheral is always available, without needing to take
    // it out of RESET.

    // Bring IO bank 0 out of reset so we can configure some pins.
    p.RESETS.reset.modify(|_, w| w.io_bank0().clear_bit());
    while !p.RESETS.reset_done.read().io_bank0().bit() {}

    // Expose SIO on diagnostic pins.
    p.IO_BANK0.gpio[LED_PIN as usize].gpio_ctrl.write(|w| w.funcsel().sio());
    p.IO_BANK0.gpio[SETUP_PIN as usize].gpio_ctrl.write(|w| w.funcsel().sio());
    p.IO_BANK0.gpio[BUFF_PIN as usize].gpio_ctrl.write(|w| w.funcsel().sio());
    p.IO_BANK0.gpio[RESET_PIN as usize].gpio_ctrl.write(|w| w.funcsel().sio());
    for pin in EP_PIN {
        p.IO_BANK0.gpio[pin as usize].gpio_ctrl.write(|w| w.funcsel().sio());
    }

    // Make them outputs.
    p.SIO.gpio_oe_set.write(|w| unsafe { w.bits(
            1 << LED_PIN
            | 1 << SETUP_PIN
            | 1 << RESET_PIN
            | 1 << BUFF_PIN
    )});
    for pin in EP_PIN {
        p.SIO.gpio_oe_set.write(|w| unsafe { w.bits(1 << pin) });
    }


    //////////////////////////////////////////////////////////////////////////
    // Clock configuration. Switch over to the crystal oscillator, enable the
    // PLL, boost clocks to target frequency.
    //
    // XOSC and CLOCKS do not need to be taken out of RESET.

    // Start XOSC.
    p.XOSC.ctrl.write(|w| unsafe { w.bits(0xfab_aa0) });
    // Wait for it to stabilize.
    while !p.XOSC.status.read().stable().bit() {}
    // Switch ref clk from ROSC to XOSC. By default, sys clk is derived from
    // this, so this switches the entire system glitchlessly.
    p.CLOCKS.clk_ref_ctrl.write(|w| w.src().xosc_clksrc());
    // Wait for mux to switch.
    while p.CLOCKS.clk_ref_selected.read().bits() != (1 << 2) {}
    // Turn off ROSC. We're now running at a crystal-governed 12MHz.
    p.ROSC.ctrl.write(|w| unsafe { w.bits(0xd1e_fa4) });

    // Bring up the system PLL to generate a higher frequency for the CPU. This
    // step is not _strictly_ necessary for getting USB working, but it sure
    // makes us respond faster.
    //
    // Begin by bringing PLL_SYS out of reset.
    p.RESETS.reset.modify(|_, w| w.pll_sys().clear_bit());
    while !p.RESETS.reset_done.read().pll_sys().bit() {}
    // Program PLL for 132 MHz clock:
    //
    // REFDIV = 1
    // FBDIV = 132 => VCO freq = 1584 MHz
    //
    p.PLL_SYS.cs.write(|w| unsafe {
        w.refdiv().bits(1)
    });
    p.PLL_SYS.fbdiv_int.write(|w| unsafe {
        w.fbdiv_int().bits(132)
    });
    p.PLL_SYS.pwr.write(|w|
        w.pd().clear_bit()
            .vcopd().clear_bit()
    );
    // Wait for lock.
    while !p.PLL_SYS.cs.read().lock().bit() {}
    // Set up post dividers to enable output.
    //
    // POSTDIV1 * POSTDIV2 = 12,
    // so PLL SYS output = 1584 / 12 = 132 MHz.
    p.PLL_SYS.prim.write(|w| unsafe {
        w.postdiv1().bits(6)
            .postdiv2().bits(2)
    });
    p.PLL_SYS.pwr.modify(|_, w| w.postdivpd().clear_bit());

    // Switch sysclk to be derived from PLL_SYS.
    p.CLOCKS.clk_sys_ctrl.write(|w| w.auxsrc().clksrc_pll_sys());
    p.CLOCKS.clk_sys_ctrl.modify(|_, w| w.src().clksrc_clk_sys_aux());
    while p.CLOCKS.clk_sys_selected.read().bits() != (1 << 1) {}
    // We are now running at 132 MHz.

    // Bring up clk peri to track clk sys (at 132 MHz). This isn't used by the
    // USB peripheral but _is_ used by the UART, and I tend to want the UART, so
    // I'm leaving this here. Feel free to ignore it.
    p.CLOCKS.clk_peri_ctrl.write(|w|
        w.auxsrc().clk_sys()
            .enable().set_bit()
    );

    // Bring PLL_USB up to 48MHz. PLL_USB is clocked from refclk, which we've
    // already moved over to the 12MHz XOSC. We just need to make it x4 that
    // clock.
    //
    // PLL_USB out of reset:
    p.RESETS.reset.modify(|_, w| w.pll_usb().clear_bit());
    while !p.RESETS.reset_done.read().pll_usb().bit() {}
    // Configure it:
    //
    // REFDIV = 1
    // FBDIV = 100 => FOUTVCO = 1200 MHz
    p.PLL_USB.cs.write(|w| unsafe {
        w.refdiv().bits(1)
    });
    p.PLL_USB.fbdiv_int.write(|w| unsafe {
        w.fbdiv_int().bits(100)
    });
    p.PLL_USB.pwr.write(|w|
        w.pd().clear_bit()
            .vcopd().clear_bit()
    );
    // Wait for lock.
    while !p.PLL_USB.cs.read().lock().bit() {}
    // Set up post dividers to enable output.
    //
    // POSTDIV1 = POSTDIV2 = 5
    // PLL_USB FOUT = 1200 MHz / 25 = 48 MHz
    p.PLL_USB.prim.write(|w| unsafe {
        w.postdiv1().bits(5)
            .postdiv2().bits(5)
    });
    p.PLL_USB.pwr.modify(|_, w| w.postdivpd().clear_bit());

    // Switch usbclk to be derived from PLLUSB.
    p.CLOCKS.clk_usb_ctrl.write(|w| w.auxsrc().clksrc_pll_usb()
        .enable().set_bit());

    // We now have the stable 48MHz reference clock required for USB.

    //////////////////////////////////////////////////////////////////////////
    // USB configuration.

    // Bring USB out of reset.
    p.RESETS.reset.modify(|_, w| w.usbctrl().set_bit());
    p.RESETS.reset.modify(|_, w| w.usbctrl().clear_bit());
    while !p.RESETS.reset_done.read().usbctrl().bit() {}

    // Clear the control portion of DPRAM. This may not be necessary -- the
    // datasheet is ambiguous -- but the C examples do it, and so do we.
    p.USBCTRL_DPRAM.setup_packet_low.write(|w| unsafe { w.bits(0) });
    p.USBCTRL_DPRAM.setup_packet_high.write(|w| unsafe { w.bits(0) });
    for epc in &p.USBCTRL_DPRAM.ep_control {
        epc.write(|w| unsafe { w.bits(0) });
    }
    for epb in &p.USBCTRL_DPRAM.ep_buffer_control {
        epb.write(|w| unsafe { w.bits(0) });
    }

    // Mux the controller to the onboard USB PHY. I was surprised that there are
    // alternatives to this, but, there are.
    p.USBCTRL_REGS.usb_muxing.write(|w| w.to_phy().set_bit()
        // This bit is also set in the SDK example, without any discussion. It's
        // undocumented (being named does not count as being documented).
        .softcon().set_bit()
    );

    // Force VBUS detect. Not all RP2040 boards wire up VBUS detect, which would
    // let us detect being plugged into a host (the Pi Pico, to its credit,
    // does). For maximum compatibility, we'll set the hardware to always
    // pretend VBUS has been detected.
    p.USBCTRL_REGS.usb_pwr.write(|w| w.vbus_detect().set_bit()
        .vbus_detect_override_en().set_bit()
    );

    // Enable controller in device mode.
    p.USBCTRL_REGS.main_ctrl.write(|w| w
        .controller_en().set_bit()
        .host_ndevice().clear_bit()
    );

    // Request to have an interrupt (which really just means setting a bit in
    // the `buff_status` register) every time a buffer moves through EP0.
    p.USBCTRL_REGS.sie_ctrl.write(|w| w.ep0_int_1buf().set_bit());

    // Enable interrupts (bits set in the `ints` register) for other conditions
    // we use:
    p.USBCTRL_REGS.inte.write(|w| w
        // A buffer is done.
        .buff_status().set_bit()
        // The host has reset us.
        .bus_reset().set_bit()
        // We've gotten a setup request on EP0.
        .setup_req().set_bit()
    );

    //////////////////////////////////////////////////////////////////////////
    // Initialize, and get references to, USB SRAM.
    //
    // This is some serious "Rust memory model vs machine reality" stuff that
    // you can skip by if you're mostly interested in how to operate the USB
    // peripheral.

    let buffers = {
        // Declare static buffers in sections that, thanks to the linker script
        // used to compile this firmware, get assigned to the USB SRAM area up
        // past 0x5000_0000.
        //
        // The buffers are declared as...
        //
        // - MaybeUninit, because the Rust startup code isn't going to
        //   initialize these parts of RAM for us, so we will find them filled
        //   with arbitrary garbage.
        //
        // - UnsafeCell, because we're going to be updating them through shared
        //   references, sometimes, when the hardware isn't looking. This would
        //   be a great thing to build an abstraction around, but clearly I
        //   haven't bothered.
        //
        // - And then fixed size arrays of 64 bytes. Technically it's no big
        //   deal to have different endpoint buffers be different sizes, but
        //   making them all consistently 64 bytes simplified some of this demo.

        // EP0 buffer0 and buffer1 are at fixed locations.
        #[link_section = ".usb_ep0_buffer0"]
        #[used]
        static mut USB_EP0_BUFFER0: MaybeUninit<UnsafeCell<[u8; 64]>> = MaybeUninit::uninit();

        #[link_section = ".usb_ep0_buffer1"]
        #[used]
        static mut USB_EP0_BUFFER1: MaybeUninit<UnsafeCell<[u8; 64]>> = MaybeUninit::uninit();

        // The rest of the buffers we'll allocate ourselves.
        #[link_section = ".usb_buffers"]
        #[used]
        static mut USB_BUFFERS: [MaybeUninit<UnsafeCell<[u8; 64]>>; 16] = {
            // This is a hack for initializing a static array with a
            // const-but-not-Copy expression. It shouldn't work but it totally
            // does.
            const XXX: MaybeUninit<UnsafeCell<[u8; 64]>> = MaybeUninit::uninit();
            [XXX; 16]
        };

        // Use `unsafe` to get exclusive references to those mutable statics.
        //
        // Safety:
        //
        // 1. We're in `main` and not in a loop. The `cortex_m_rt` support (the
        //    `#[entry]` attribute used above) ensures that you don't call
        //    `main` reentrantly from safe code. This means we can assume that
        //    code will pass this point at most once, ensuring our references
        //    are unique.
        //
        // 2. We haven't told the hardware to interact with these buffers yet,
        //    so we're not racing anyone.
        //
        // This would be a fantastic place to use the First-Mover Allocator
        // Pattern, but, Cortex-M0 lacks the atomics to do it the obvious way.
        let ep0_buffer0 = unsafe { &mut USB_EP0_BUFFER0 };
        let ep0_buffer1 = unsafe { &mut USB_EP0_BUFFER1 };
        let rest = unsafe { &mut USB_BUFFERS };

        // Initialize the buffers. This is a formality, since we're going to
        // overwrite them. Rust's rules on initialization are not super clear in
        // cases like this, so I'm being conservative.
        *ep0_buffer0 = MaybeUninit::new(UnsafeCell::new([0; 64]));
        *ep0_buffer1 = MaybeUninit::new(UnsafeCell::new([0; 64]));
        for b in &mut *rest {
            *b = MaybeUninit::new(UnsafeCell::new([0; 64]));
        }

        // Package all that up into _shared_ references that can be passed
        // around and used from safe code. This requires using `assume_init_ref`
        // on the MaybeUninits to assert that they've been initialized (above),
        // and I'm doing something similar with pointer casting for the array
        // because `assume_init_array` and friends are not stable as of
        // 2022-04-28.
        Buffers {
            ep0_buffer0: unsafe { ep0_buffer0.assume_init_ref() },
            ep0_buffer1: unsafe { ep0_buffer1.assume_init_ref() },
            rest: unsafe { &*(rest as *const _ as *const _) },
        }
    };

    //////////////////////////////////////////////////////////////////////////
    // Back to USB setup.

    // Set up endpoints.
    for ep in &DEVICE_CONFIGURATION.endpoints {
        // EP0 doesn't have an endpoint control index; only process the other
        // endpoints here.
        if let Some(epci) = ep.endpoint_control_index {
            // We need to compute the offset from the base of USB SRAM to the
            // buffer we're choosing, because that's how the peripheral do.
            let buf_base = buffers.get(ep.data_buffer_index) as *const _ as u32;
            let dpram_base = rp2040_pac::USBCTRL_DPRAM::ptr() as u32;
            let dpram_offset = buf_base - dpram_base;
            // The offset _should_ fit in a u16, but if we've gotten something
            // wrong in the past few lines, a common symptom will be integer
            // overflow producing a Very Large Number, which will then cause us
            // to panic here, instead of producing nonsense:
            let dpram_offset = u16::try_from(dpram_offset).unwrap();

            // Configure the endpoint!
            p.USBCTRL_DPRAM.ep_control[epci].write(|w| unsafe {
                w.enable().set_bit()
                    // Please set the corresponding bit in buff_status when a
                    // buffer is done, thx.
                    .interrupt_per_buff().set_bit()
                    // Select bulk vs control.
                    .endpoint_type().bits(ep.descriptor.attributes)
                    // And, designate our buffer by its offset.
                    .buffer_address().bits(dpram_offset)
            });
        }
    }

    // Present full-speed device by enabling pullup on DP. This is the point
    // where the host will notice our presence.
    p.USBCTRL_REGS.sie_ctrl.modify(|_, w| w.pullup_en().set_bit());

    //////////////////////////////////////////////////////////////////////////
    // Main loop.
    //
    // We'll keep some state in Plain Old Local Variables:

    // When the host gives us a new address, we can't just slap it into
    // registers right away, because we have to do an acknowledgement step using
    // our _old_ address. Instead, we'll store `Some(the_address)` here and then
    // handle it in a later step (below).
    let mut new_address = None;
    // Flag recording whether the host has configured us with a
    // `SetConfiguration` message.
    let mut configured = false;
    // Flag recording whether we've set up buffer transfers after being
    // configured.
    let mut started = false;
    // Some scratch space that we'll use for things like preparing string
    // descriptors for transmission.
    let mut tmp: [u8; 64] = [0; 64];

    loop {
        // Check which interrupt flags are set.
        let ints = p.USBCTRL_REGS.ints.read();

        // Setup request received?
        if ints.setup_req().bit() {
            raise_pin(&p.SIO, LED_PIN);
            raise_pin(&p.SIO, SETUP_PIN);

            // Clear the status flag (write-one-to-clear).
            p.USBCTRL_REGS.sie_status.write(|w| w.setup_rec().set_bit());

            // This assumes that the setup packet is arriving on EP0, our
            // control endpoint. Which it should be. We don't have any other
            // Control endpoints.

            // Copy the setup packet out of its dedicated buffer at the base of
            // USB SRAM. The PAC models this buffer as two 32-bit registers,
            // which is, like, not _wrong_ but slightly awkward since it means
            // we can't just treat it as bytes. Instead, copy it out to a byte
            // array.
            let mut setup_packet = [0; 8];
            setup_packet[..4].copy_from_slice(&p.USBCTRL_DPRAM.setup_packet_low.read().bits().to_le_bytes());
            setup_packet[4..].copy_from_slice(&p.USBCTRL_DPRAM.setup_packet_high.read().bits().to_le_bytes());
            // Reinterpret as a setup packet.
            let setup = zerocopy::LayoutVerified::<_, UsbSetupPacket>::new(&setup_packet[..]).unwrap().into_ref();

            // Reset PID to 1 for EP0 IN. Every DATA packet we send in response
            // to an IN on EP0 needs to use PID DATA1, and this line will ensure
            // that.
            EP0_IN_CFG.next_pid_1.store(true, Ordering::Relaxed);

            // Attempt to parse the request type and request into one of our
            // known enum values, and then inspect them. (These will return None
            // if we get an unexpected numeric value.)
            let reqty = UsbDir::from_u8(setup.request_type);
            let req = UsbSetupRequest::from_u8(setup.request);
            match (reqty, req) {
                (Some(UsbDir::Out), Some(UsbSetupRequest::SetAddress)) => {
                    // The new address is in the bottom 8 bits of the setup
                    // packet value field. Store it for use later.
                    let addr = setup.value.get() as u8;
                    new_address = Some(addr);
                    // The address will actually get set later, we have
                    // to use address 0 to send a status response.
                    usb_acknowledge_out_request(
                        &p.USBCTRL_DPRAM,
                        &buffers,
                    );
                }
                (Some(UsbDir::Out), Some(UsbSetupRequest::SetConfiguration)) => {
                    // We only have one configuration, and it doesn't really
                    // mean anything to us -- more of a formality. All we do in
                    // response to this is:
                    configured = true;
                    usb_acknowledge_out_request(
                        &p.USBCTRL_DPRAM,
                        &buffers,
                    );
                }
                (Some(UsbDir::Out), _) => {
                    // This is sort of a hack, but: if we get any other kind of
                    // OUT, just acknowledge it with the same zero-length status
                    // phase that we use for control transfers that we _do_
                    // understand. This keeps the host from spinning forever
                    // while we NAK.
                    //
                    // This behavior copied shamelessly from the C example.
                    usb_acknowledge_out_request(
                        &p.USBCTRL_DPRAM,
                        &buffers,
                    );
                }
                (Some(UsbDir::In), Some(UsbSetupRequest::GetDescriptor)) => {
                    // Identify the requested descriptor type, which is in the
                    // _top_ 8 bits of value.
                    let descriptor_type = UsbDescType::from_u16(setup.value.get() >> 8);
                    match descriptor_type {
                        Some(UsbDescType::Device) => {
                            // TODO: this sure looks like a duplicate, but it's
                            // a duplicate that was present in the C
                            // implementation.
                            EP0_IN_CFG.next_pid_1.store(true, Ordering::Relaxed);

                            // Configure EP0 IN to send the device descriptor
                            // when it's next asked.
                            usb_start_tx(
                                &p.USBCTRL_DPRAM,
                                &buffers,
                                &EP0_IN_CFG,
                                DEVICE_CONFIGURATION.device_descriptor.as_bytes(),
                            );
                        }
                        Some(UsbDescType::Config) => {
                            // Config descriptor requests are slightly unusual.
                            // We can respond with just our config descriptor,
                            // but we can _also_ append our interface and
                            // endpoint descriptors to the end, saving some
                            // round trips. We'll choose to do this if the
                            // number of bytes the host will accept (in the
                            // `length` field) is large enough.
                            let mut used = 0;
                            {
                                let cd = DEVICE_CONFIGURATION.config_descriptor.as_bytes();
                                tmp[used..used + cd.len()].copy_from_slice(cd);
                                used += cd.len();
                            }

                            if usize::from(setup.length.get()) > used {
                                // Do the rest!
                                //
                                // This is slightly incorrect because the host
                                // might have asked for a number of bytes in
                                // between the size of a config descriptor, and
                                // the amount we're going to send back. However,
                                // in practice, the host always asks for either
                                // (1) the exact size of a config descriptor, or
                                // (2) 64 bytes, and this all fits in 64 bytes.
                                {
                                    let id = DEVICE_CONFIGURATION.interface_descriptor.as_bytes();
                                    tmp[used..used + id.len()].copy_from_slice(id);
                                    used += id.len();
                                }
                                for ep in &DEVICE_CONFIGURATION.endpoints[2..] {
                                    let ed = ep.descriptor.as_bytes();
                                    tmp[used..used + ed.len()].copy_from_slice(ed);
                                    used += ed.len();
                                }
                            }

                            // Set up EP0 IN to send the stuff we just composed.
                            usb_start_tx(
                                &p.USBCTRL_DPRAM,
                                &buffers,
                                &EP0_IN_CFG,
                                &tmp[..used],
                            );
                        }
                        Some(UsbDescType::String) => {
                            // String descriptor index is in bottom 8 bits of
                            // `value`.
                            let i = usize::from(setup.value.get() & 0xFF);
                            let bytes = if i == 0 {
                                // Special index 0 requests the language
                                // descriptor.
                                DEVICE_CONFIGURATION.lang_descriptor
                            } else {
                                // Otherwise, set up one of our strings.
                                let s = DEVICE_CONFIGURATION.descriptor_strings[i - 1];
                                let len = 2 + s.len();
                                tmp[0] = len as u8;
                                tmp[1] = 0x03;

                                tmp[2..len].copy_from_slice(s);

                                &tmp[..len]
                            };
                            // Set up EP0 IN to send whichever thing we just
                            // decided on.
                            usb_start_tx(
                                &p.USBCTRL_DPRAM,
                                &buffers,
                                &EP0_IN_CFG,
                                bytes,
                            );

                        }
                        Some(UsbDescType::Interface) => {
                            // We don't expect the host to send this because we
                            // delivered our interface descriptor with the
                            // config descriptor.
                            //
                            // Should probably implement it, though, because
                            // otherwise the host will be unhappy. TODO.
                            //
                            // Note that the C example gets away with ignoring
                            // this.
                        }
                        Some(UsbDescType::Endpoint) => {
                            // Same deal as interface descriptors above.
                        }
                        _ => {
                            // Unrecognized descriptor type. We should probably
                            // indicate an error. Instead we'll just ignore it,
                            // because this doesn't happen in practice.
                        }
                    }
                }
                (Some(UsbDir::In), _) => {
                    // Other IN request. Ignore.
                }
                _ => {
                    // Unexpected request type or request bits. This can totally
                    // happen (yay, hardware!) but is rare in practice. Ignore
                    // it.
                }
            }
            lower_pin(&p.SIO, SETUP_PIN);
        }

        // Events on one or more buffers? (In practice, always one.)
        if ints.buff_status().bit() {
            raise_pin(&p.SIO, LED_PIN);
            raise_pin(&p.SIO, BUFF_PIN);

            let orig_bufbits = p.USBCTRL_REGS.buff_status.read().bits();

            // Let's try being super tricky and iterating through set bits with
            // Cleverness(tm).
            // Become mutable so that I can clear bit as I handle 'em. Keep the
            // original around so I can use it to clear the register in a bit.
            let mut bufbits = orig_bufbits;

            while bufbits != 0 {
                // Who's still outstanding? Find their bit index by counting how
                // many LSBs are zero.
                let lowbit_index = bufbits.trailing_zeros();
                // Remove their bit from our set.
                let lowbit = 1 << lowbit_index;
                bufbits ^= lowbit;

                // Here we exploit knowledge of the ordering of buffer control
                // registers in the peripheral. Each endpoint has a pair of
                // registers, so we can determine the endpoint number by:
                let epnum = (lowbit_index >> 1) as u8;
                // Of the pair, the IN endpoint comes first, followed by OUT, so
                // we can get the direction by:
                let dir = if lowbit_index & 1 == 0 { UsbDir::In } else { UsbDir::Out };

                // Signal activity if there's a pin for this endpoint.
                if let Some(&pin) = EP_PIN.get(epnum as usize) {
                    raise_pin(&p.SIO, pin);
                }

                let ep_addr = dir.endpoint(epnum);
                // Process the buffer-done event.
                let data = {
                    // Scan the device table to figure out which endpoint struct
                    // corresponds to this address. We could use a smarter
                    // method here, but in practice, the number of endpoints is
                    // small so a linear scan doesn't kill us.
                    let ep = DEVICE_CONFIGURATION.endpoints.iter()
                        .find(|ep| ep.descriptor.endpoint_address == ep_addr)
                        .expect("buffer event for unknown EP?!");
                    // Read the buffer control register to check status.
                    let bc = p.USBCTRL_DPRAM.ep_buffer_control[ep.buffer_control_index].read();
                    // We should only get here if we've been notified that
                    // the buffer is ours again. This is indicated by the hw
                    // _clearing_ the AVAILABLE bit.
                    //
                    // This ensures that we can return a shared reference to
                    // the databuffer contents without races.
                    assert!(!bc.available_0().bit());

                    // Cool. Checks out.

                    // Get a pointer to the buffer in USB SRAM. This is the
                    // buffer _contents_ within the UnsafeCell. See the
                    // safety comments below.
                    let epbuffer = buffers.get(ep.data_buffer_index);
                    let epbuffer = epbuffer.get() as *const u8;

                    // Get the actual length of the data, which may be less
                    // than the buffer size.
                    let len = bc.length_0().bits() as usize;

                    // Make a byte slice pointing into USB SRAM.
                    //
                    // Safety: because we always use the same data buffer
                    // with the same endpoint / buffer control register, and
                    // because we have ensured that the available bit in the
                    // buffer control register is not set, we can be
                    // confident that we're not racing the hardware.
                    //
                    // You can _definitely_ abuse this code in a way that
                    // produces multiple slices pointing to the same buffer,
                    // but since these are shared references, that's
                    // perfectly legal.
                    let data = unsafe {
                        core::slice::from_raw_parts(
                            epbuffer,
                            len,
                        )
                    };

                    data
                };

                // Perform any required action on the data. For OUT, the `data`
                // will be whatever was sent by the host. For IN, it's a copy of
                // whatever we sent.
                match ep_addr {
                    EP0_IN_ADDR => {
                        // We use this opportunity to finish the delayed
                        // SetAddress request, if there is one:
                        if let Some(a) = new_address.take() {
                            // Change our address:
                            p.USBCTRL_REGS.addr_endp.write(|w| unsafe { w.address().bits(a) });
                        } else {
                            // Otherwise, we've just finished sending
                            // something to the host. We expect an ensuing
                            // status phase where the host sends us (via EP0
                            // OUT) a zero-byte DATA packet, so, set that
                            // up:
                            usb_start_rx(
                                &p.USBCTRL_DPRAM,
                                &EP0_OUT_CFG,
                                0,
                            );
                        }
                    }
                    EP1_OUT_ADDR => {
                        // We've gotten data from the host on our custom
                        // EP1! Set up EP2 to repeat it.
                        usb_start_tx(
                            &p.USBCTRL_DPRAM,
                            &buffers,
                            &EP2_IN_CFG,
                            data,
                        );
                    }
                    EP2_IN_ADDR => {
                        // The host has collected the data we repeated onto
                        // EP2! Set up to receive more data on EP1.
                        usb_start_rx(
                            &p.USBCTRL_DPRAM,
                            &EP1_OUT_CFG,
                            64,
                        );
                    }
                    _ => {
                        // In practice, the only other endpoint that
                        // might successfully complete a transfer is EP0
                        // OUT, which is handled by the control logic
                        // above.
                    }
                }

                if let Some(&pin) = EP_PIN.get(epnum as usize) {
                    lower_pin(&p.SIO, pin);
                }
            }
            // Acknowledge all buffers, since we handled them all above.
            p.USBCTRL_REGS.buff_status.write(|w| unsafe { w.bits(orig_bufbits) });

            lower_pin(&p.SIO, BUFF_PIN);
        }

        // Has the host signaled a bus reset?
        if ints.bus_reset().bit() {
            raise_pin(&p.SIO, LED_PIN);
            raise_pin(&p.SIO, RESET_PIN);
            // Acknowledge by writing the write-one-to-clear status bit.
            p.USBCTRL_REGS.sie_status.write(|w| w.bus_reset().set_bit());

            // Reset our state.
            new_address = None;
            configured = false;
            started = false;
            p.USBCTRL_REGS.addr_endp.write(|w| unsafe { w.address().bits(0) });

            lower_pin(&p.SIO, RESET_PIN);
        }

        // If we have been configured but haven't reached this point yet, set up
        // our custom EP1 OUT to receive whatever data the host wants to send.
        if configured && !started {
            started = true;
            usb_start_rx(
                &p.USBCTRL_DPRAM,
                &EP1_OUT_CFG,
                64,
            );
        }
        lower_pin(&p.SIO, LED_PIN);
    } // end of main loop.
}

////////////////////////////////////////////////////////////////////////////
// Assorted utility functions factored out of main.
//
// I've only done this for functions that are used more than once. It would be
// reasonable to factor more stuff out, but, again, this is intended to be read,
// not maintained.

/// Configure the next IN on EP0 to deliver a successful-but-empty payload. This
/// is used in the status phase of control transactions.
fn usb_acknowledge_out_request(
    usb_dp_regs: &rp2040_pac::USBCTRL_DPRAM,
    ep_buffers: &Buffers,
) {
    usb_start_tx(
        usb_dp_regs,
        ep_buffers,
        &EP0_IN_CFG,
        &[], // <-- see, empty buffer
    );
}

/// Configures a given endpoint to send data (device-to-host, IN) when the host
/// next asks for it.
///
/// The contents of `buffer` will be _copied_ into USB SRAM, so you can
/// reuse `buffer` immediately after this returns. No need to wait for the
/// packet to be sent.
fn usb_start_tx(
    usb_dp_regs: &rp2040_pac::USBCTRL_DPRAM,
    ep_buffers: &Buffers,
    ep: &UsbEndpointConfiguration,
    buffer: &[u8],
) {
    // It is technically possible to support longer buffers but this demo
    // doesn't bother.
    assert!(buffer.len() <= 64);
    // You should only be calling this on IN endpoints.
    assert!(UsbDir::of_endpoint_addr(ep.descriptor.endpoint_address) == UsbDir::In);

    let epbuffer = ep_buffers.get(ep.data_buffer_index);
    let epbuffer = epbuffer.get() as *mut u8;
    // Safety: we're depositing data into the buffer. We never produce a &
    // reference into the buffer, so the main safety issue here is whether the
    // buffer pointer itself is valid -- which it should be, since we just got
    // it from `buffers`.
    unsafe {
        epbuffer.copy_from_nonoverlapping(
            buffer.as_ptr(),
            buffer.len(),
        );
    }

    // Check which DATA0/1 PID this endpoint is expecting next.
    let np = ep.next_pid_1.load(Ordering::Relaxed);
    // Configure the IN:
    usb_dp_regs.ep_buffer_control[ep.buffer_control_index].write(|w| unsafe {
        w
            // DATA0/1, depending
            .pid_0().bit(np)
            // We have put data in.
            .full_0().set_bit()
            // The data is for the computer to use now.
            .available_0().set_bit()
            // There are this many bytes.
            .length_0().bits(buffer.len() as u16)
    });
    // Flip the DATA0/1 PID for the next transmission.
    ep.next_pid_1.store(!np, Ordering::Relaxed);
}

fn usb_start_rx(
    usb_dp_regs: &rp2040_pac::USBCTRL_DPRAM,
    ep: &UsbEndpointConfiguration,
    len: usize,
) {
    // It is technically possible to support longer buffers but this demo
    // doesn't bother.
    assert!(len <= 64);
    // You should only be calling this on OUT endpoints.
    assert!(UsbDir::of_endpoint_addr(ep.descriptor.endpoint_address) == UsbDir::Out);

    // Check which DATA0/1 PID this endpoint is expecting next.
    let np = ep.next_pid_1.load(Ordering::Relaxed);
    // Configure the OUT:
    usb_dp_regs.ep_buffer_control[ep.buffer_control_index].write(|w| unsafe {
        w
            // DATA0/1, depending
            .pid_0().bit(np)
            // Buffer is NOT full, we want the computer to fill it plz.
            .full_0().clear_bit()
            // It is, however, available to be filled.
            .available_0().set_bit()
            // Up to this many bytes.
            .length_0().bits(len as u16)
    });
    // Flip the DATA0/1 PID for the next receive.
    ep.next_pid_1.store(!np, Ordering::Relaxed);
}

////////////////////////////////////////////////////////////////////////////
// Driver support stuctures.

struct UsbDeviceConfiguration {
    device_descriptor: &'static UsbDeviceDescriptor,
    interface_descriptor: &'static UsbInterfaceDescriptor,
    config_descriptor: &'static UsbConfigurationDescriptor,
    lang_descriptor: &'static [u8],
    descriptor_strings: &'static [&'static [u8]],

    endpoints: [&'static UsbEndpointConfiguration; 4],
}

struct UsbEndpointConfiguration {
    descriptor: &'static UsbEndpointDescriptor,

    /// Index of this endpoint's control register in the `ep_control` array.
    ///
    /// TODO: this can be derived from the endpoint address, perhaps it should
    /// be.
    endpoint_control_index: Option<usize>,
    /// Index of this endpoint's buffer control register in the
    /// `ep_buffer_control` array.
    ///
    /// TODO this, too, can be derived.
    buffer_control_index: usize,

    /// Index of this endpoint's data buffer in the array of data buffers
    /// allocated from DPRAM. This can be arbitrary, and endpoints can even
    /// share buffers if you're careful.
    data_buffer_index: usize,

    /// Keeps track of which DATA PID (DATA0/DATA1) is expected on this endpoint
    /// next. If `true`, we're expecting `DATA1`, otherwise `DATA0`.
    ///
    /// This is an `AtomicBool` so that we can update it in a `static`, not
    /// because we actually rely on its atomicity for any reason.
    next_pid_1: AtomicBool,
}

/// Buffer pointers, once they're prepared and initialized.
struct Buffers {
    /// Fixed EP0 Buffer0, defined by the hardware.
    ep0_buffer0: &'static UnsafeCell<[u8; 64]>,
    /// Fixed EP0 Buffer1, defined by the hardware and NOT USED in this driver.
    ep0_buffer1: &'static UnsafeCell<[u8; 64]>,
    /// Remaining buffer pool.
    rest: &'static [UnsafeCell<[u8; 64]>; 16],
}

impl Buffers {
    /// Gets a buffer corresponding to a `data_buffer_index` in a
    /// `UsbEndpointConfiguration`.
    pub fn get(&self, i: usize) -> &UnsafeCell<[u8; 64]> {
        match i {
            0 => self.ep0_buffer0,
            1 => self.ep0_buffer1,
            x => &self.rest[x - 2],
        }
    }
}

// Handy constants for the endpoints we use here
const EP0_IN_ADDR: u8 = UsbDir::In.endpoint(0);
const EP0_OUT_ADDR: u8 = UsbDir::Out.endpoint(0);
const EP1_OUT_ADDR: u8 = UsbDir::Out.endpoint(1);
const EP2_IN_ADDR: u8 = UsbDir::In.endpoint(2);

// Configuration of our device:
static DEVICE_CONFIGURATION: UsbDeviceConfiguration = UsbDeviceConfiguration {
    device_descriptor: &UsbDeviceDescriptor {
        length: core::mem::size_of::<UsbDeviceDescriptor>() as u8,
        descriptor_type: UsbDescType::Device,
        bcd_usb: U16::from_bytes(u16::to_le_bytes(0x0110)),
        device_class: 0,
        device_subclass: 0,
        device_protocol: 0,
        max_packet_size0: 64,
        vendor: U16::from_bytes(u16::to_le_bytes(0)),
        product: U16::from_bytes(u16::to_le_bytes(1)),
        bcd_device: U16::from_bytes(u16::to_le_bytes(0)),
        manufacturer_s: 1,
        product_s: 2,
        serial_s: 0,
        num_configurations: 1,
    },
    interface_descriptor: &UsbInterfaceDescriptor {
        length: core::mem::size_of::<UsbInterfaceDescriptor>() as u8,
        descriptor_type: UsbDescType::Interface,
        interface_number: 0,
        alternate_setting: 0,
        num_endpoints: 2,
        interface_class: 0xFF,
        interface_subclass: 0,
        interface_protocol: 0,
        interface_s: 0,
    },
    config_descriptor: &UsbConfigurationDescriptor {
        length: core::mem::size_of::<UsbConfigurationDescriptor>() as u8,
        descriptor_type: UsbDescType::Config,
        total_length: U16::from_bytes(u16::to_le_bytes(
            core::mem::size_of::<UsbConfigurationDescriptor>() as u16
            + core::mem::size_of::<UsbInterfaceDescriptor>() as u16
            + core::mem::size_of::<UsbEndpointDescriptor>() as u16
            + core::mem::size_of::<UsbEndpointDescriptor>() as u16
        )),
        num_interfaces: 1,
        configuration_value: 1,
        configuration_s: 0,
        attributes: 0xC0,
        max_power: 0x32,
    },
    lang_descriptor: &[4, 0x03, 0x09, 0x04],
    descriptor_strings: &[
        // Look at these gross UTF-16 strings!
        b"R\0a\0s\0p\0b\0e\0r\0r\0y\0 \0P\0i\0",
        b"P\0i\0c\0o\0 \0T\0e\0s\0t\0 \0D\0e\0v\0i\0c\0e\0",
    ],
    endpoints: [
        &EP0_OUT_CFG,
        &EP0_IN_CFG,
        &EP1_OUT_CFG,
        &EP2_IN_CFG,
    ],
};

static EP0_OUT_CFG: UsbEndpointConfiguration = UsbEndpointConfiguration {
    descriptor: &UsbEndpointDescriptor {
        length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
        descriptor_type: UsbDescType::Endpoint,
        endpoint_address: EP0_OUT_ADDR,
        attributes: UsbTransferType::Control as u8,
        max_packet_size: U16::from_bytes(u16::to_le_bytes(64)),
        interval: 0,
    },
    endpoint_control_index: None,
    buffer_control_index: 1,
    data_buffer_index: 0,
    next_pid_1: AtomicBool::new(false),
};
static EP0_IN_CFG: UsbEndpointConfiguration = UsbEndpointConfiguration {
    descriptor: &UsbEndpointDescriptor {
        length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
        descriptor_type: UsbDescType::Endpoint,
        endpoint_address: EP0_IN_ADDR,
        attributes: UsbTransferType::Control as u8,
        max_packet_size: U16::from_bytes(u16::to_le_bytes(64)),
        interval: 0,
    },
    endpoint_control_index: None,
    buffer_control_index: 0,
    data_buffer_index: 0,
    next_pid_1: AtomicBool::new(false),
};
static EP1_OUT_CFG: UsbEndpointConfiguration = UsbEndpointConfiguration {
    descriptor: &UsbEndpointDescriptor {
        length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
        descriptor_type: UsbDescType::Endpoint,
        endpoint_address: EP1_OUT_ADDR,
        attributes: UsbTransferType::Bulk as u8,
        max_packet_size: U16::from_bytes(u16::to_le_bytes(64)),
        interval: 0,
    },
    endpoint_control_index: Some(1),
    buffer_control_index: 3,
    data_buffer_index: 2,
    next_pid_1: AtomicBool::new(false),
};
static EP2_IN_CFG: UsbEndpointConfiguration = UsbEndpointConfiguration {
    descriptor: &UsbEndpointDescriptor {
        length: core::mem::size_of::<UsbEndpointDescriptor>() as u8,
        descriptor_type: UsbDescType::Endpoint,
        endpoint_address: EP2_IN_ADDR,
        attributes: UsbTransferType::Bulk as u8,
        max_packet_size: U16::from_bytes(u16::to_le_bytes(64)),
        interval: 0,
    },
    endpoint_control_index: Some(2),
    buffer_control_index: 4,
    data_buffer_index: 3,
    next_pid_1: AtomicBool::new(false),
};


////////////////////////////////////////////////////////////////////////////
// USB structure and constant definitions.

/// USB deals in two different transfer directions, called OUT (host-to-device)
/// and IN (device-to-host). In the vast majority of cases, OUT is represented
/// by a 0 byte, and IN by an `0x80` byte.
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
enum UsbDir {
    Out = 0,
    In = 0x80,
}

impl UsbDir {
    pub const fn endpoint(self, num: u8) -> u8 {
        num | self as u8
    }

    pub const fn of_endpoint_addr(addr: u8) -> Self {
        if addr & Self::In as u8 != 0 {
            Self::In
        } else {
            Self::Out
        }
    }
}

/// Layout of an 8-byte USB SETUP packet.
#[repr(C)]
#[derive(Debug, AsBytes, FromBytes, Unaligned)]
struct UsbSetupPacket {
    /// Request type; in practice, this is always either OUT (host-to-device) or
    /// IN (device-to-host), whose values are given in the `UsbDir` enum.
    request_type: u8,
    /// Request. Standard setup requests are in the `UsbSetupRequest` enum.
    /// Devices can extend this with additional types as long as they don't
    /// conflict.
    request: u8,
    /// A simple argument of up to 16 bits, specific to the request.
    value: U16<LittleEndian>,
    /// Not used in the requests we support.
    index: U16<LittleEndian>,
    /// If data will be transferred after this request (in the direction given
    /// by `request_type`), this gives the number of bytes (OUT) or maximum
    /// number of bytes (IN).
    length: U16<LittleEndian>,
}

/// The types of USB SETUP requests that we understand.
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
enum UsbSetupRequest {
    /// Asks the device to send a certain descriptor back to the host. Always
    /// used on an IN request.
    GetDescriptor = 0x06,
    /// Notifies the device that it's being moved to a different address on the
    /// bus. Always an OUT.
    SetAddress = 0x05,
    /// Configures a device by choosing one of the options listed in its
    /// descriptors. Always an OUT.
    SetConfiguration = 0x09,
}

/// Describes a device. This is the most broad description in USB and is
/// typically the first thing the host asks for.
#[repr(C)]
#[derive(Debug, AsBytes)]
struct UsbDeviceDescriptor {
    /// Length of this structure, must be 18.
    length: u8,
    /// Type of this descriptor, must be `Device`.
    descriptor_type: UsbDescType,
    /// Version of the device descriptor / USB protocol, in binary-coded
    /// decimal. This is typically `0x01_10` for USB 1.1.
    bcd_usb: U16<LittleEndian>,
    /// Class of device, giving a broad functional area.
    device_class: u8,
    /// Subclass of device, refining the class.
    device_subclass: u8,
    /// Protocol within the subclass.
    device_protocol: u8,
    /// Maximum unit of data this device can move.
    max_packet_size0: u8,
    /// ID of product vendor.
    vendor: U16<LittleEndian>,
    /// ID of product.
    product: U16<LittleEndian>,
    /// Device version number, as BCD again.
    bcd_device: U16<LittleEndian>,
    /// Index of manufacturer name in string descriptor table.
    manufacturer_s: u8,
    /// Index of product name in string descriptor table.
    product_s: u8,
    /// Index of serial number in string descriptor table.
    serial_s: u8,
    /// Number of configurations supported by this device.
    num_configurations: u8,
}

/// Description of a single available device configuration.
#[repr(C)]
#[derive(Debug, AsBytes)]
struct UsbConfigurationDescriptor {
    /// Length of this structure, must be 9.
    length: u8,
    /// Type of this descriptor, must be `Config`.
    descriptor_type: UsbDescType,
    /// Total length of all descriptors in this configuration, concatenated.
    /// This will include this descriptor, plus at least one interface
    /// descriptor, plus each interface descriptor's endpoint descriptors.
    total_length: U16<LittleEndian>,
    /// Number of interface descriptors in this configuration.
    num_interfaces: u8,
    /// Number to use when requesting this configuration via a
    /// `SetConfiguration` request.
    configuration_value: u8,
    /// Index of this configuration's name in the string descriptor table.
    configuration_s: u8,
    /// Bit set of device attributes:
    ///
    /// - Bit 7 should be set (indicates that device can be bus powered in USB
    /// 1.0).
    /// - Bit 6 indicates that the device can be self-powered.
    /// - Bit 5 indicates that the device can signal remote wakeup of the host
    /// (like a keyboard).
    /// - The rest are reserved and should be zero.
    attributes: u8,
    /// Maximum device power consumption in units of 2mA.
    max_power: u8,
}

/// Description of an interface within a configuration.
#[repr(C)]
#[derive(Debug, AsBytes)]
struct UsbInterfaceDescriptor {
    /// Length of this structure, must be 9.
    length: u8,
    /// Type of this descriptor, must be `Interface`.
    descriptor_type: UsbDescType,
    /// ID of this interface.
    interface_number: u8,
    /// Allows a single `interface_number` to have several alternate interface
    /// settings, where each alternate increments this field. Normally there's
    /// only one, and `alternate_setting` is zero.
    alternate_setting: u8,
    /// Number of endpoint descriptors in this interface.
    num_endpoints: u8,
    /// Interface class code, distinguishing the type of interface.
    interface_class: u8,
    /// Interface subclass code, refining the class of interface.
    interface_subclass: u8,
    /// Protocol within the interface class/subclass.
    interface_protocol: u8,
    /// Index of interface name within string descriptor table.
    interface_s: u8,
}

/// Describes an endpoint within an interface.
#[repr(C)]
#[derive(Debug, AsBytes)]
struct UsbEndpointDescriptor {
    /// Length of this struct, must be 7.
    length: u8,
    /// Type of this descriptor, must be `Endpoint`.
    descriptor_type: UsbDescType,
    /// Address of this endpoint, where the bottom 4 bits give the endpoint
    /// number (0..15) and the top bit distinguishes IN (1) from OUT (0).
    endpoint_address: u8,
    /// Endpoint attributes; the most relevant part is the bottom 2 bits, which
    /// control the transfer type using the values from `UsbTransferType`.
    attributes: u8,
    /// Maximum packet size this endpoint can accept/produce.
    max_packet_size: U16<LittleEndian>,
    /// Interval for polling interrupt/isochronous endpoints (which we don't
    /// currently support) in milliseconds.
    interval: u8,
}

/// Types of USB descriptor.
#[derive(Copy, Clone, Debug, FromPrimitive, AsBytes)]
#[repr(u8)]
enum UsbDescType {
    Device = 0x01,
    Config = 0x02,
    String = 0x03,
    Interface = 0x04,
    Endpoint = 0x05,
}

/// Types of transfer that can be indicated by the `attributes` field on
/// `UsbEndpointDescriptor`.
#[derive(Copy, Clone, Debug, FromPrimitive, AsBytes)]
#[repr(u8)]
enum UsbTransferType {
    Control = 0,
    Bulk = 2,
}

////////////////////////////////////////////////////////////////////////////
// Assorted RP2040 support stuff.

/// Raise the GPIO pin with index `pin`.
///
/// The pin must already be configured as an SIO output.
#[inline(always)]
fn raise_pin(sio: &rp2040_pac::SIO, pin: u8) {
    sio.gpio_out_set.write(|w| unsafe { w.bits(1 << pin) });
}

/// Lower the GPIO pin with index `pin`.
///
/// The pin must already be configured as an SIO output.
#[inline(always)]
fn lower_pin(sio: &rp2040_pac::SIO, pin: u8) {
    sio.gpio_out_clr.write(|w| unsafe { w.bits(1 << pin) });
}

// Include the bootloader. We just include it as a data constant. This is
// relying on the good graces of `used`, which we all know is a great way to get
// your foot shot off later. Should revisit this.
cfg_if::cfg_if! {
    if #[cfg(feature = "target-feather")] {
        // The Adafruit board uses a cute little GigaDevices flash chip that
        // requires a different bootloader, not totally sure why.
        #[link_section = ".boot_loader"]
        #[used]
        static BOOT2: [u8; 256] = *include_bytes!("rustboot-gd25q64.bin");
    } else if #[cfg(feature = "target-pico")] {
        // The Pi Pico uses a Winbond W25Q080 chip.
        #[link_section = ".boot_loader"]
        #[used]
        static BOOT2: [u8; 256] = *include_bytes!("rustboot-w25q080.bin");
    } else {
        compiler_error!("must include one target-* feature");
    }
}
