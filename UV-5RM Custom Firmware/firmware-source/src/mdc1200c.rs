#![allow(dead_code)]

pub const MDC1200_FEC_K: usize = 7;
pub const MDC1200_SYNC: [u8; 5] = [0x07, 0x09, 0x2A, 0x44, 0x6F];

/// Standard MDC1200 CRC-16 (reflected 0x8408 polynomial, init 0x0000, final XOR 0xFFFF)
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

/// Differential XOR modulation for 1200 bps MSK transmission
pub fn xor_modulation(data: &mut [u8]) {
    let mut prev_bit: u8 = 0;
    for byte in data.iter_mut() {
        let input = *byte;
        let mut out: u8 = 0;
        for bit_num in (0..8).rev() {
            let new_bit = (input >> bit_num) & 1;
            if new_bit != prev_bit {
                out |= 1 << bit_num;
            }
            prev_bit = new_bit;
        }
        *byte = out ^ 0xFF;
    }
}

/// R=1/2 K=7 Convolutional encoder and bit interleaver
pub fn encode_data(data: &mut [u8]) {
    let mut shift_reg: u8 = 0;
    for i in 0..MDC1200_FEC_K {
        let bi = data[i];
        let mut bo: u8 = 0;
        for bit_num in 0..8 {
            shift_reg = (shift_reg << 1) | ((bi >> bit_num) & 1);
            let parity = ((shift_reg >> 6) ^ (shift_reg >> 5) ^ (shift_reg >> 2) ^ shift_reg) & 1;
            bo |= parity << bit_num;
        }
        data[MDC1200_FEC_K + i] = bo;
    }

    let mut interleaved = [0u8; (MDC1200_FEC_K * 2) * 8];
    let mut k = 0;
    for i in 0..(MDC1200_FEC_K * 2) {
        let b = data[i];
        for bit_num in 0..8 {
            interleaved[k] = (b >> bit_num) & 1;
            k += 16;
            if k >= interleaved.len() {
                k -= interleaved.len() - 1;
            }
        }
    }

    k = 0;
    for i in 0..(MDC1200_FEC_K * 2) {
        let mut b: u8 = 0;
        for bit_num in (0..8).rev() {
            if interleaved[k] != 0 {
                b |= 1 << bit_num;
            }
            k += 1;
        }
        data[i] = b;
    }
}

/// Builds a complete 19-byte modulated MDC1200 single packet (5-byte sync + 14-byte FEC payload)
pub fn encode_single_packet(out: &mut [u8; 19], op: u8, arg: u8, unit_id: u16) -> usize {
    out[0..5].copy_from_slice(&MDC1200_SYNC);

    let payload = &mut out[5..19];
    payload[0] = op;
    payload[1] = arg;
    payload[2] = ((unit_id >> 8) & 0xFF) as u8;
    payload[3] = (unit_id & 0xFF) as u8;

    let crc = compute_crc(&payload[0..4]);
    payload[4] = (crc & 0xFF) as u8;
    payload[5] = ((crc >> 8) & 0xFF) as u8;
    payload[6] = 0;

    encode_data(payload);
    xor_modulation(out);
    19
}
