//! The modulus encoder of 6.4.1 and its inverse: K bits to twelve K_i by a
//! mixed radix, after the "sign" of the frame is differentially encoded so
//! that a line that inverts the signal costs one frame and no more.

fn product(moduli: &[u16; 12]) -> u128 {
    moduli.iter().map(|&m| u128::from(m.max(1))).product()
}

/// s(f): whether R is above (M - 1)/2, in integers.
fn sign(r: u128, m: u128) -> bool {
    2 * r > m - 1
}

/// 6.4.1, one data frame at a time; `d` is d(f - 1).
#[derive(Debug, Clone)]
pub struct ModulusEncoder {
    d: bool,
}

impl ModulusEncoder {
    pub fn new(d_prev: bool) -> Self {
        Self { d: d_prev }
    }

    /// K bits, b0 first in time, to K0..K11.
    pub fn encode(&mut self, bits: &[bool], moduli: &[u16; 12]) -> [u16; 12] {
        // 1) R = b0 + b1 x 2 + ... + b(K-1) x 2^(K-1).
        let r = bits.iter().enumerate().fold(0u128, |r, (i, &b)| r | u128::from(b) << i);
        let m = product(moduli);
        // 2), 3) and 4): the sign, differentially; R0 from d(f - 1).
        let r0 = if self.d { m - 1 - r } else { r };
        self.d ^= sign(r, m);
        // 5) Ki = Ri mod Mi; R(i+1) = (Ri - Ki)/Mi.
        let mut ri = r0;
        moduli.map(|mi| {
            let mi = u128::from(mi.max(1));
            let ki = ri % mi;
            ri /= mi;
            ki as u16
        })
    }
}

/// The digital modem's inverse of [`ModulusEncoder`].
#[derive(Debug, Clone)]
pub struct ModulusDecoder {
    d: bool,
}

impl ModulusDecoder {
    pub fn new(d_prev: bool) -> Self {
        Self { d: d_prev }
    }

    /// K0..K11 back to the K bits. K_i outside 0..M_i are taken modulo M_i.
    pub fn decode(&mut self, ks: &[u16; 12], moduli: &[u16; 12], k: u32) -> Vec<bool> {
        let m = product(moduli);
        let r0 = ks.iter().zip(moduli).rev().fold(0u128, |r, (&ki, &mi)| {
            let mi = u128::from(mi.max(1));
            r * mi + u128::from(ki) % mi
        });
        let r = if self.d { m - 1 - r0 } else { r0 };
        self.d ^= sign(r, m);
        (0..k).map(|i| i < 128 && r >> i & 1 == 1).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(v: &[u8]) -> Vec<bool> { v.iter().map(|&b| b == 1).collect() }
    const POW2: [u16; 12] = [2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1];

    #[test]
    fn two_frames_by_hand() {
        let mut e = ModulusEncoder::new(false);
        // R 13 > 7.5: s 1, d 1; d(f-1) 0 so R0 = R.
        assert_eq!(e.encode(&bits(&[1, 0, 1, 1]), &POW2), [1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
        // R 4: s 0, d 1; d(f-1) 1 so R0 = 15 - 4 = 11.
        assert_eq!(e.encode(&bits(&[0, 0, 1, 0]), &POW2), [1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn mixed_radix_by_hand() {
        let moduli = [3, 5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
        // M 15: R 7 = (M - 1)/2 is not above it, s 0; K0 = 7 mod 3, K1 = 2.
        assert_eq!(ModulusEncoder::new(false).encode(&bits(&[1, 1, 1]), &moduli)[..2], [1, 2]);
    }

    fn frames(n: usize) -> Vec<Vec<bool>> {
        let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
        (0..n).map(|_| (0..48).map(|_| { x ^= x << 13; x ^= x >> 7; x ^= x << 17; x & 1 == 1 }).collect()).collect()
    }

    #[test]
    fn decode_inverts_encode() {
        let (mut e, mut d) = (ModulusEncoder::new(false), ModulusDecoder::new(false));
        for f in frames(500) {
            let ks = e.encode(&f, &[16; 12]);
            assert_eq!(d.decode(&ks, &[16; 12], 48), f);
        }
    }

    #[test]
    fn a_wrong_starting_sign_costs_one_frame() {
        let (mut e, mut d) = (ModulusEncoder::new(false), ModulusDecoder::new(true));
        for (i, f) in frames(20).into_iter().enumerate() {
            let got = d.decode(&e.encode(&f, &[16; 12]), &[16; 12], 48);
            assert_eq!(got == f, i > 0, "frame {i}");
        }
    }
}
