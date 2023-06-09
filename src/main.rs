#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI1])]
mod app {
    use systick_monotonic::Systick;

    use core::mem::MaybeUninit;

    use stm32f4xx_hal::{
        gpio::{Alternate, NoPin, Output, Pin, PushPull, Speed},
        otg_fs::{UsbBus, UsbBusType, USB},
        pac::SPI1,
        prelude::*,
        spi::{Spi, *},
    };

    use embedded_hal::spi::{Mode, Phase, Polarity};

    use usb_device::{bus::UsbBusAllocator, prelude::*};
    use usbd_serial::SerialPort;

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use sh1106::{prelude::*, Builder};

    // Setting this monotonic as the default
    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1000>;

    type SckPin = Pin<'A', 5, Alternate<5>>;
    type MosiPin = Pin<'A', 7, Alternate<5>>;
    type DcPin = Pin<'A', 3, Output>;
    type CsPin = Pin<'A', 4, Output>;
    type SpiType = Spi<SPI1, (SckPin, NoPin, MosiPin), false>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        serial: SerialPort<'static, UsbBus<USB>>,
        display: GraphicsMode<SpiInterface<SpiType, DcPin, CsPin>>,
    }

    #[init(local = [ep_memory: [u32; 1024] = [0; 1024], usb_bus: MaybeUninit<UsbBusAllocator<UsbBusType>> = MaybeUninit::uninit()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono: Systick<1000> = Systick::new(ctx.core.SYST, 96_000_000);

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(96.MHz()).freeze();

        let gpioc = ctx.device.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();

        let gpioa = ctx.device.GPIOA.split();
        let usb = USB {
            usb_global: ctx.device.OTG_FS_GLOBAL,
            usb_device: ctx.device.OTG_FS_DEVICE,
            usb_pwrclk: ctx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        let usb_bus = ctx.local.usb_bus;
        let usb_bus = usb_bus.write(UsbBus::new(usb, ctx.local.ep_memory));

        let serial = SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let sck = gpioa.pa5.into_alternate().speed(Speed::VeryHigh);
        let mosi = gpioa
            .pa7
            .into_alternate()
            .speed(Speed::VeryHigh)
            .internal_pull_up(true);
        let dc = gpioa.pa3.into_push_pull_output();
        let cs = gpioa.pa4.into_push_pull_output();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };

        let spi = Spi::new(
            ctx.device.SPI1,
            (sck, NoMiso {}, mosi),
            mode,
            8.MHz(),
            &clocks,
        );

        let display: GraphicsMode<_> = Builder::new().connect_spi(spi, dc, cs).into();

        blink_task::spawn().ok();
        display_task::spawn().ok();

        (
            Shared {},
            Local {
                led,
                usb_dev,
                serial,
                display,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [display])]
    fn display_task(ctx: display_task::Context) {
        let display = ctx.local.display;

        display.init().unwrap();
        display.flush().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        display.flush().unwrap();
    }

    #[task(local = [led])]
    fn blink_task(ctx: blink_task::Context) {
        ctx.local.led.toggle();
        blink_task::spawn_after(500.millis().into()).ok();
    }

    #[task(binds=OTG_FS, local = [serial, usb_dev])]
    fn on_usb(ctx: on_usb::Context) {
        let serial = ctx.local.serial;
        if !ctx.local.usb_dev.poll(&mut [serial]) {
            return;
        }
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }
                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }
}
