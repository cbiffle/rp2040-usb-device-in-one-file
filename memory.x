MEMORY {
    /* stage0 boot loads stage1 from first 256 bytes of flash. */
    RPBOOT : ORIGIN = 0x10000000, LENGTH = 256
    /* our program immediately follows it. */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 256

    RAM : ORIGIN = 0x20000000, LENGTH = 264K

    USBDPRAM_EP0_B0 : ORIGIN = 0x50100100, LENGTH = 64
    USBDPRAM_EP0_B1 : ORIGIN = 0x50100140, LENGTH = 64
    USBDPRAM_BUFFERS : ORIGIN = 0x50100180, LENGTH = 4096 - 0x180
}

SECTIONS {
    .boot_loader ORIGIN(RPBOOT): {
        KEEP(*(.boot_loader*));
    } >RPBOOT
} INSERT BEFORE .text;

SECTIONS {
    .usbdpramep0b0 ORIGIN(USBDPRAM_EP0_B0) (NOLOAD) : {
        KEEP(*(.usb_ep0_buffer0));
    } >USBDPRAM_EP0_B0
    .usbdpramep0b1 ORIGIN(USBDPRAM_EP0_B1) (NOLOAD) : {
        KEEP(*(.usb_ep0_buffer1));
    } >USBDPRAM_EP0_B1
    .usbdprambuffers ORIGIN(USBDPRAM_BUFFERS) (NOLOAD) : {
        KEEP(*(.usb_buffers));
    } >USBDPRAM_BUFFERS
} INSERT AFTER .uninit;
