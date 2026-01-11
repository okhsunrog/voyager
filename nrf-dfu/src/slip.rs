//! SLIP (Serial Line Internet Protocol) framing
//!
//! Used by the Nordic DFU protocol for packet framing over serial.

// SLIP special bytes
pub const END: u8 = 0xC0;
pub const ESC: u8 = 0xDB;
pub const ESC_END: u8 = 0xDC;
pub const ESC_ESC: u8 = 0xDD;

/// Encode data using SLIP framing
pub fn encode(data: &[u8], out: &mut Vec<u8>) {
    for &byte in data {
        match byte {
            END => {
                out.push(ESC);
                out.push(ESC_END);
            }
            ESC => {
                out.push(ESC);
                out.push(ESC_ESC);
            }
            _ => out.push(byte),
        }
    }
    out.push(END);
}

/// Decode SLIP-encoded data
/// Returns the decoded data, stopping at END byte
#[cfg(test)]
pub fn decode(data: &[u8]) -> Result<Vec<u8>, &'static str> {
    let mut out = Vec::new();
    let mut iter = data.iter();

    while let Some(&byte) = iter.next() {
        match byte {
            END => return Ok(out),
            ESC => match iter.next() {
                Some(&ESC_END) => out.push(END),
                Some(&ESC_ESC) => out.push(ESC),
                Some(&_invalid) => {
                    return Err("invalid byte following ESC");
                }
                None => return Err("unexpected end after ESC"),
            },
            _ => out.push(byte),
        }
    }

    Err("no END byte found")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_simple() {
        let mut out = Vec::new();
        encode(&[0x01, 0x02, 0x03], &mut out);
        assert_eq!(out, vec![0x01, 0x02, 0x03, END]);
    }

    #[test]
    fn test_encode_escape_end() {
        let mut out = Vec::new();
        encode(&[END], &mut out);
        assert_eq!(out, vec![ESC, ESC_END, END]);
    }

    #[test]
    fn test_encode_escape_esc() {
        let mut out = Vec::new();
        encode(&[ESC], &mut out);
        assert_eq!(out, vec![ESC, ESC_ESC, END]);
    }

    #[test]
    fn test_decode_simple() {
        let decoded = decode(&[0x01, 0x02, 0x03, END]).unwrap();
        assert_eq!(decoded, vec![0x01, 0x02, 0x03]);
    }

    #[test]
    fn test_roundtrip() {
        let original = vec![0x00, END, ESC, 0xFF, 0x42];
        let mut encoded = Vec::new();
        encode(&original, &mut encoded);
        let decoded = decode(&encoded).unwrap();
        assert_eq!(decoded, original);
    }
}
