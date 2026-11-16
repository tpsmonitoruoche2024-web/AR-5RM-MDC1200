#![allow(dead_code)]

pub const MDC1200_FEC_K: usize = 7; // R=1/2 K=7 convolutional coder
pub const MDC1200_RX_BUFFER_LEN: usize = 5 + (MDC1200_FEC_K * 2); // 19 bytes

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mdc1200OpCode {
    EmergencyAlarm = 0x00, // arg 0x81
    PttId = 0x01,          // arg 0x80 (PTT ID) or 0x00 (POST ID)
    RemoteMonitor = 0x11,  // arg 0x8A
    EmergencyAck = 0x20,   // arg 0x00
    StatusReq = 0x22,      // arg 0x06
    RadioEnable = 0x2B,    // arg 0x0C (Enable) or 0x00 (Disable)
    RadioCheckAck = 0x30,  // arg 0x00
    CallAlert = 0x35,      // arg 0x89
    StsXx = 0x46,          // arg 0xXX
    MsgXx = 0x47,          // arg 0xXX
    RadioCheck = 0x63,     // arg 0x85
}

pub const MDC1200_PRE_AMBLE: [u8; 2] = [0x00, 0xFF]; //
pub const MDC1200_SYNC: [u8; 5] = [0x07, 0x09, 0x2A, 0x44, 0x6F]; //
pub const MDC1200_SYNC_SUC_XOR: [u8; 5] = [0xFB, 0x72, 0x40, 0x99, 0xA7]; //

pub const MDC_NUM_ADD: u16 = 0x1FFF; //
pub const MAX_CONTACT_NUM: u8 = 21; //

/// Hardware abstraction trait for the FD6818 / BK4819 transceiver registers
pub trait Mdc1200RadioHal {
    fn read_register(&mut self, reg: u8) -> u16;
    fn write_register(&mut self, reg: u8, val: u16);
}

#[derive(Copy, Clone, Debug)]
pub struct Mdc1200DecodedPacket {
    pub op: u8,
    pub arg: u8,
    pub unit_id: u16,
}

#[derive(Copy, Clone)]
struct Mdc1200RxStream {
    bit: u8,
    prev_bit: u8,
    xor_bit: u8,
    shift_reg: u64,
    bit_count: usize,
    stage: u8,
    inverted_sync: bool,
    data_index: usize,
    data: [u8; 40],
}

impl Mdc1200RxStream {
    const fn new() -> Self {
        Self {
            bit: 0,
            prev_bit: 0,
            xor_bit: 0,
            shift_reg: 0,
            bit_count: 0,
            stage: 0,
            inverted_sync: false,
            data_index: 0,
            data: [0; 40],
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

pub struct Mdc1200State {
    pub op: u8,
    pub arg: u8,
    pub unit_id: u16,
    pub fleet_id: u16,
    pub rx_ready_tick_500ms: u8,
    pub update_display: bool,
    rx_buffer: [u8; MDC1200_RX_BUFFER_LEN],
    rx_buffer_index: usize,
    rx: Mdc1200RxStream,
}

impl Mdc1200State {
    pub const fn new() -> Self {
        Self {
            op: 0,
            arg: 0,
            unit_id: 0,
            fleet_id: 0,
            rx_ready_tick_500ms: 0,
            update_display: false,
            rx_buffer: [0; MDC1200_RX_BUFFER_LEN],
            rx_buffer_index: 0,
            rx: Mdc1200RxStream::new(),
        }
    }

    pub fn init(&mut self) {
        self.reset_rx(); //
    }

    pub fn reset_rx(&mut self) {
        self.rx.reset(); //
    }

    /// Processes incoming demodulated FSK bytes across fragmented chunks
    pub fn process_rx_data(&mut self, buffer: &[u8]) -> Option<Mdc1200DecodedPacket> {
        for &rx_byte in buffer {
            for bit in (0..8).rev() {
                self.rx.prev_bit = self.rx.bit;
                self.rx.bit = (rx_byte >> bit) & 1;
                self.rx.xor_bit = (self.rx.xor_bit ^ self.rx.bit) & 1;
                self.rx.shift_reg = (self.rx.shift_reg << 1) | (self.rx.xor_bit as u64);
                self.rx.bit_count += 1;

                if self.rx.stage == 0 {
                    // Looking for 40-bit sync word
                    const SYNC_BIT_OK_THRESHOLD: u32 = 32;

                    if self.rx.bit_count >= 40 {
                        let sync_nor: u64 = 0x0709_2A44_6F;
                        let sync_inv: u64 = 0xFF_FFFF_FFFF ^ sync_nor;

                        let diff_nor = (sync_nor ^ self.rx.shift_reg) & 0xFF_FFFF_FFFF;
                        let diff_inv = (sync_inv ^ self.rx.shift_reg) & 0xFF_FFFF_FFFF;

                        let nor_count = 40 - diff_nor.count_ones();
                        let inv_count = 40 - diff_inv.count_ones();

                        if nor_count >= SYNC_BIT_OK_THRESHOLD || inv_count >= SYNC_BIT_OK_THRESHOLD {
                            self.rx.inverted_sync = inv_count > nor_count;
                            self.rx.data_index = 0;
                            self.rx.bit_count = 0;
                            self.rx.stage = 1;
                        }
                    }
                    continue;
                }

                // Payload collection stage
                if self.rx.bit_count < 8 {
                    continue;
                }

                self.rx.bit_count = 0;
                self.rx.data[self.rx.data_index] = (self.rx.shift_reg & 0xFF) as u8;
                self.rx.data_index += 1;

                if self.rx.data_index < (MDC1200_FEC_K * 2) {
                    continue;
                }

                if !decode_data(&mut self.rx.data[..(MDC1200_FEC_K * 2)]) {
                    self.reset_rx();
                    continue;
                }

                let packet = Mdc1200DecodedPacket {
                    op: self.rx.data[0],
                    arg: self.rx.data[1],
                    unit_id: ((self.rx.data[2] as u16) << 8) | (self.rx.data[3] as u16),
                };

                self.reset_rx();
                return Some(packet);
            }
        }
        None
    }

    /// Handles BK4819 / FD6818 FSK RX interrupts
    pub fn solve_sign<H: Mdc1200RadioHal>(&mut self, hal: &mut H, interrupt_bits: u16) {
        const REG_02_FSK_RX_SYNC: u16 = 1 << 11;
        const REG_02_FSK_FIFO_ALMOST_FULL: u16 = 1 << 12;
        const REG_02_FSK_RX_FINISHED: u16 = 1 << 13;

        let rx_sync = (interrupt_bits & REG_02_FSK_RX_SYNC) != 0;
        let rx_fifo_almost_full = (interrupt_bits & REG_02_FSK_FIFO_ALMOST_FULL) != 0;
        let rx_finished = (interrupt_bits & REG_02_FSK_RX_FINISHED) != 0;

        let rx_sync_flags = hal.read_register(0x0B);
        let rx_sync_neg = (rx_sync_flags & (1 << 7)) != 0;

        if rx_sync {
            self.rx_buffer_index = 0;
            let xor_mask = if rx_sync_neg { 0xFF } else { 0x00 };
            for &b in &MDC1200_SYNC_SUC_XOR {
                if self.rx_buffer_index < MDC1200_RX_BUFFER_LEN {
                    self.rx_buffer[self.rx_buffer_index] = b ^ xor_mask;
                    self.rx_buffer_index += 1;
                }
            }
        }

        if rx_fifo_almost_full {
            let count = hal.read_register(0x5E) & 0x07;
            let word_mask = if rx_sync_neg { 0xFFFF } else { 0x0000 };

            for _ in 0..count {
                let word = hal.read_register(0x5F) ^ word_mask;
                if self.rx_buffer_index < MDC1200_RX_BUFFER_LEN {
                    self.rx_buffer[self.rx_buffer_index] = (word & 0xFF) as u8;
                    self.rx_buffer_index += 1;
                }
                if self.rx_buffer_index < MDC1200_RX_BUFFER_LEN {
                    self.rx_buffer[self.rx_buffer_index] = (word >> 8) as u8;
                    self.rx_buffer_index += 1;
                }
            }

            if self.rx_buffer_index >= MDC1200_RX_BUFFER_LEN {
                let mut packet_buf = [0u8; MDC1200_RX_BUFFER_LEN];
                packet_buf.copy_from_slice(&self.rx_buffer);

                if let Some(pkt) = self.process_rx_data(&packet_buf) {
                    self.op = pkt.op;
                    self.arg = pkt.arg;
                    self.unit_id = pkt.unit_id;
                    self.rx_ready_tick_500ms = 10;
                    self.update_display = true;
                }
                self.rx_buffer_index = 0;
            }
        }

        if rx_finished {
            let fsk_reg59 = hal.read_register(0x59)
                & !((1 << 15) | (1 << 14) | (1 << 12) | (1 << 11));
            hal.write_register(0x59, (1 << 15) | (1 << 14) | fsk_reg59);
            hal.write_register(0x59, (1 << 12) | fsk_reg59);
        }
    }
}

/// Standard MDC1200 CRC-16 (reflected 0x8408 / 0xA001 polynomial, init 0x0000, final XOR 0xFFFF)
pub fn compute_crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if (crc & 0x0001) != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

/// R=1/2 K=7 Convolutional FEC error correction
pub fn error_correction(data: &mut [u8]) {
    let mut shift_reg: u8 = 0;
    let mut syn: u8 = 0;

    for i in 0..MDC1200_FEC_K {
        let bi = data[i];
        for bit_num in 0..8 {
            let mut k: u32 = 0;

            shift_reg = (shift_reg << 1) | ((bi >> bit_num) & 1);
            let b = ((shift_reg >> 6) ^ (shift_reg >> 5) ^ (shift_reg >> 2) ^ shift_reg) & 1;
            let parity_bit = (data[i + MDC1200_FEC_K] >> bit_num) & 1;
            syn = (syn << 1) | (b ^ parity_bit);

            if (syn & 0x80) != 0 { k += 1; }
            if (syn & 0x20) != 0 { k += 1; }
            if (syn & 0x04) != 0 { k += 1; }
            if (syn & 0x02) != 0 { k += 1; }

            if k >= 3 {
                let mut ii = i as isize;
                let mut bn = (bit_num as isize) - 7;
                if bn < 0 {
                    bn += 8;
                    ii -= 1;
                }
                if ii >= 0 {
                    data[ii as usize] ^= 1u8 << (bn as u8);
                }
                syn ^= 0xA6;
            }
        }
    }
}

/// De-interleaving and FEC + CRC verification
pub fn decode_data(data: &mut [u8]) -> bool {
    let mut encoded = [0u8; MDC1200_FEC_K * 2];
    let mut deinterleaved = [0u8; (MDC1200_FEC_K * 2) * 8];

    let mut k = 0;
    for i in 0..16 {
        for m in 0..MDC1200_FEC_K {
            let n = (m * 16) + i;
