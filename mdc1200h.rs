#![allow(dead_code)]

/// R=1/2 K=7 convolutional coder
pub const MDC1200_FEC_K: usize = 7;

/// Receive buffer size: 5-byte sync + 14-byte FEC payload (19 bytes total)
pub const MDC1200_RX_BUFFER_LEN: usize = 5 + (MDC1200_FEC_K * 2);

/// EEPROM address storing the number of saved MDC1200 contacts
pub const MDC_NUM_ADD: u16 = 0x1FFF;

/// Maximum number of stored MDC1200 contact aliases
pub const MAX_CONTACT_NUM: u8 = 21;

/// Standard MDC1200 sync word
pub const MDC1200_SYNC: [u8; 5] = [0x07, 0x09, 0x2A, 0x44, 0x6F];

/// XOR-modulated MDC1200 sync word used for hardware FSK sync matching
pub const MDC1200_SYNC_SUC_XOR: [u8; 5] = [0xFB, 0x72, 0x40, 0x99, 0xA7];

/// MDC1200 Operation Codes (`mdc1200_op_code_t`)
///
/// Reference Argument (`arg`) pairings:
/// - `0x00` (`0x81`): Emergency alarm
/// - `0x20` (`0x00`): Emergency alarm ack
/// - `0x01` (`0x80`): PTT ID (Pre-ID)
/// - `0x01` (`0x00`): POST ID
/// - `0x11` (`0x8A`): Remote Monitor
/// - `0x22` (`0x06`): Status Request
/// - `0x2B` (`0x0C`): Radio Enable
/// - `0x2B` (`0x00`): Radio Disable
/// - `0x35` (`0x89`): Call Alert
/// - `0x46` (`0xXX`): Status XX
/// - `0x47` (`0xXX`): Message XX
/// - `0x63` (`0x85`): Radio Check
/// - `0x30` (`0x00`): Radio Check ack
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mdc1200OpCode {
    PttId = 0x01,
    RemoteMonitor = 0x11,
    StatusReq = 0x22,
    RadioEnable = 0x2B,
    CallAlert = 0x35,
    StsXx = 0x46,
    MsgXx = 0x47,
    RadioCheck = 0x63,
}

impl Mdc1200OpCode {
    /// Post-ID shares opcode 0x01 with PTT ID (differentiated by arg = 0x00 vs 0x80)
    pub const POST_ID: u8 = 0x01;

    /// Radio Disable shares opcode 0x2B with Radio Enable (differentiated by arg = 0x00 vs 0x0C)
    pub const RADIO_DISABLE: u8 = 0x2B;
}

/// Decoded MDC1200 single-packet payload fields
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Mdc1200Packet {
    pub op: u8,
    pub arg: u8,
    pub unit_id: u16,
}

/// Global/shared MDC1200 runtime state variables corresponding to the `extern` declarations in `mdc1200.h`
#[derive(Copy, Clone, Debug)]
pub struct Mdc1200Globals {
    pub mdc1200_op: u8,
    pub mdc1200_arg: u8,
    pub mdc1200_unit_id: u16,
    pub mdc1200_fleet_id: u16,
    pub mdc1200_rx_buffer: [u8; MDC1200_RX_BUFFER_LEN],
    pub mdc1200_rx_buffer_index: usize,
    pub mdc1200_rx_ready_tick_500ms: u8,
    pub contact_num: u8,
}

impl Mdc1200Globals {
    pub const fn new() -> Self {
        Self {
            mdc1200_op: 0,
            mdc1200_arg: 0,
            mdc1200_unit_id: 0,
            mdc1200_fleet_id: 0,
            mdc1200_rx_buffer: [0; MDC1200_RX_BUFFER_LEN],
            mdc1200_rx_buffer_index: 0,
            mdc1200_rx_ready_tick_500ms: 0,
            contact_num: 0,
        }
    }
}

/// Interface trait mirroring the function prototypes declared in `mdc1200.h`
pub trait Mdc1200Interface {
    fn init(&mut self);
    fn reset_rx(&mut self);
    fn process_rx_data(&mut self, buffer: &[u8]) -> Option<Mdc1200Packet>;
    fn encode_single_packet(out: &mut [u8; MDC1200_RX_BUFFER_LEN], op: u8, arg: u8, unit_id: u16) -> usize;
    fn solve_sign(&mut self, interrupt_bits: u16);
    fn contact_find(&mut self, mdc_id: u16, contact_out: &mut [u8; 15]) -> bool;
    fn fleetsync_contact_find(&mut self, fleet_id: u16, unit_id: u16, contact_out: &mut [u8; 9]) -> bool;
}
