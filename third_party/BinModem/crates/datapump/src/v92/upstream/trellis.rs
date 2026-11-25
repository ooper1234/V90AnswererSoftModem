//! The trellis side of 6.4.2-6.4.4: equivalence classes, the inverse map and
//! K_i back from an index.

use crate::v34::trellis::{convert, label};

/// E(K_i) of 6.4.2 as indices eta, -N/2 <= eta < N/2, in increasing order:
/// eta = K_i + z M_i at trellis positions 0 to 2, and at position 3
/// eta = 2 K_i + 2 z M_i + `parity`, `parity` being (eta0 + eta1 + eta2 + Y0)
/// mod 2.
pub fn class(position: usize, ki: u16, mi: u16, n: i32, parity: bool) -> Vec<i32> {
    let (base, step) = if position == 3 {
        (2 * i32::from(ki) + i32::from(parity), 2 * i32::from(mi.max(1)))
    } else {
        (i32::from(ki), i32::from(mi.max(1)))
    };
    let lowest = -n / 2;
    let first = lowest + (base - lowest).rem_euclid(step);
    (first..n / 2).step_by(step as usize).collect()
}

/// 6.4.3: the trellis frame's pairs (y(0), y(1)) and (y(2), y(3)) as the
/// odd-integer points (2y + 1) of 9.6.3.1/V.34, through its symbol-to-bit
/// converter: [Y4 Y3 Y2 Y1] as bits 3 to 0.
pub fn inverse_map(etas: [i32; 4]) -> u8 {
    let point = |a: i32, b: i32| (2 * a + 1, 2 * b + 1);
    convert(label(point(etas[0], etas[1])), label(point(etas[2], etas[3])))
}

/// K_i back from the index sent at trellis position `position`.
pub fn ki(position: usize, eta: i32, mi: u16) -> u16 {
    let mi = i32::from(mi.max(1));
    let e = if position == 3 { eta.div_euclid(2) } else { eta };
    e.rem_euclid(mi) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classes_by_hand() {
        assert_eq!(class(0, 1, 4, 16, false), vec![-7, -3, 1, 5]);
        assert_eq!(class(3, 1, 4, 16, true), vec![-5, 3]);
        assert_eq!(class(3, 1, 4, 16, false), vec![-6, 2]);
    }

    #[test]
    fn inverse_map_by_hand() {
        assert_eq!(inverse_map([0, 0, 0, 0]), 0b0000);
        // (3, 1) is label 111 and (1, 1) label 000; Table 13 row 7 column 0.
        assert_eq!(inverse_map([1, 0, 0, 0]), 0b1110);
    }

    #[test]
    fn ki_undoes_class() {
        for position in 0..4 {
            for mi in [3u16, 4, 16] {
                for k in 0..mi {
                    for parity in [false, true] {
                        let n = 2 * i32::from(mi) * 2;
                        for eta in class(position, k, mi, n, parity) {
                            assert_eq!(ki(position, eta, mi), k, "position {position} mi {mi} eta {eta}");
                        }
                    }
                }
            }
        }
    }
}
