#[derive(Debug)]
pub enum PeripheralIdentifiers {
    Supc = 0,         // Supply Controller
    Rstc = 1,         // Reset Controller
    Rtc = 2,          // Real Time Clock
    Rtt = 3,          // Real Time Timer
    Wdt = 4,          // Watchdog Timer
    Pmc = 5,          // Power Management Controller
    Efc = 6,          // Enhanced Embedded Flash Controller
    Uart0 = 7,        // Universal Asynchronous Receiver/Transmitter
    Uart1 = 8,        // Universal Asynchronous Receiver/Transmitter
    Smc = 9,          // Static Memory Controller
    Pioa = 10,        // Parallel I/O Controller A
    Piob = 11,        // Parallel I/O Controller B
    Pioc = 12,        // Parallel I/O Controller C
    Usart0 = 13,      // Universal Synchronous/Asynchronous Receiver/Transmitter,
    Usart1 = 14,      // Universal Synchronous/Asynchronous Receiver/Transmitter,
    Usart2 = 15,      // Universal Synchronous/Asynchronous Receiver/Transmitter,
    Piod = 16,        // Parallel I/O Controller A
    Pioe = 17,        // Parallel I/O Controller B
    Hsmci = 18,       // Multimedia Card Interface
    Twihs0 = 19,      // Two-wire Interface (I2C-compatible)
    Twihs1 = 20,      // Two-wire Interface (I2C-compatible)
    Spi0 = 21,        // Serial Peripheral Interface
    Ssc = 22,         // Synchronous Serial Controller
    Tc0Channel0 = 23, // 16-bit Timer Counter 0, Channel 0
    Tc0Channel1 = 24, // 16-bit Timer Counter 0, Channel 1
    Tc0Channel2 = 25, // 16-bit Timer Counter 0, Channel 2
    Tc1Channel0 = 26, // 16-bit Timer Counter 1, Channel 0
    Tc1Channel1 = 27, // 16-bit Timer Counter 1, Channel 1
    Tc1Channel2 = 28, // 16-bit Timer Counter 1, Channel 2
    Afec0 = 29,       // Analog Front-End Controller
    Dacc = 30,        // Digital-to-Analog Converter
    Pwm0 = 31,        // Pulse Width Modulation Controller
    Icm = 32,         // Integrity Check Monitor
    Acc = 33,         // Analog Comparator Controller
    Usbhs = 34,       // USB Host / Device Controller
    Mcan0 = 35,       // CAN IRQ Line 0
    Mcan0Int1 = 36,   // CAN IRQ Line 1, INT1
    Mcan1 = 37,       // CAN IRQ Line 0
    Mcan1Int1 = 38,   // CAN IRQ Line 1, INT1
    Gmac = 39,        // Ethernet MAC
    Afec1 = 40,       // Analog Front-End Controller
    Twihs2 = 41,      // Two-wire Interface (I2C-compatible)
    Spi1 = 42,        // Serial Peripheral Interface
    Qspi = 43,        // Quad I/O Serial Peripheral Interface
    Uart2 = 44,       // Universal Asynchronous Receiver/Transmitter
    Uart3 = 45,       // Universal Asynchronous Receiver/Transmitter
    Uart4 = 46,       // Universal Asynchronous Receiver/Transmitter
    Tc2Channel0 = 47, // 16-bit Timer Counter 2, Channel 0
    Tc2Channel1 = 48, // 16-bit Timer Counter 2, Channel 1
    Tc2Channel2 = 49, // 16-bit Timer Counter 2, Channel 2
    Tc3Channel0 = 50, // 16-bit Timer Counter 3, Channel 0
    Tc3Channel1 = 51, // 16-bit Timer Counter 3, Channel 1
    Tc3Channel2 = 52, // 16-bit Timer Counter 3, Channel 2
    Mlb0 = 53,        // MediaLB IRQ 0
    Mlb1 = 54,        // MediaLB IRQ 1
    Reserved0 = 55,   // Reserved
    Aes = 56,         // Advanced Encryption Standard
    Trng = 57,        // True Random Number Generator
    Xdmac = 58,       // DMA Controller
    Isi = 59,         // Image Sensor Interface
    Pwm1 = 60,        // Pulse Width Modulation Controller
    ArmIntFpu = 61, // ARM Floating Point Unit interrupt associated with OFC, UFC, IOC, DZC and IDC bits
    Sdramc = 62,    // SDRAM Controller
    RsWdt = 63,     // Reinforced Safety Watchdog Timer
    ArmIntCcw = 64, // ARM Cache ECC Warning
    ArmIntCcf = 65, // Arm Cache ECC Fault
    GmacIntQ1 = 66, // GMAC Queue 1 Interrupt signal toggled on a DMA write to the first word of each DMA data buffer associated with queue 1
    GmacIntQ2 = 67, // GMAC Queue 2 Interrupt signal toggled on a DMA write to the first word of each DMA data buffer associated with queue 2
    ArmIntIxc = 68, // Floating Point Unit Interrupt IXC associated with FPU cumulative exception bit
    I2sc0 = 69,     // Inter-IC Sound Controller
    I2sc1 = 70,     // Inter-IC Sound Controller
    GmacIntQ3 = 71, // GMAC Queue 2 Interrupt signal toggled on a DMA write to the first word of each DMA data buffer associated with queue 2
    GmacIntQ4 = 72, // GMAC Queue 2 Interrupt signal toggled on a DMA write to the first word of each DMA data buffer associated with queue 2
    GmacIntQ5 = 73, // GMAC Queue 2 Interrupt signal toggled on a DMA write to the first word of each DMA data buffer associated with queue 2
}
