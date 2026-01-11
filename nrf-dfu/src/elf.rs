//! ELF to binary conversion

use object::{Object, ObjectSection};
use std::io;

/// Convert an ELF file to a raw binary image
///
/// This extracts all loadable sections from the ELF and creates a contiguous
/// binary image suitable for flashing.
pub fn elf_to_bin(elf_data: &[u8]) -> io::Result<Vec<u8>> {
    let elf = object::File::parse(elf_data)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;

    // Find the lowest and highest addresses
    let mut min_addr = u64::MAX;
    let mut max_addr = 0u64;

    for section in elf.sections() {
        if section.size() == 0 {
            continue;
        }

        let addr = section.address();
        if addr == 0 {
            continue;
        }

        // Only include sections with data
        if let Ok(data) = section.data()
            && !data.is_empty()
        {
            min_addr = min_addr.min(addr);
            max_addr = max_addr.max(addr + section.size());
        }
    }

    if min_addr == u64::MAX {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "No loadable sections found in ELF",
        ));
    }

    // Create binary image filled with 0xFF (erased flash state)
    let size = (max_addr - min_addr) as usize;
    let mut bin = vec![0xFFu8; size];

    for section in elf.sections() {
        if section.size() == 0 {
            continue;
        }

        let addr = section.address();
        if addr == 0 {
            continue;
        }

        if let Ok(data) = section.data()
            && !data.is_empty()
        {
            let offset = (addr - min_addr) as usize;
            let end = offset + data.len();
            if end <= bin.len() {
                bin[offset..end].copy_from_slice(data);
            }
        }
    }

    log::info!(
        "Converted ELF to binary: {} bytes (0x{:08X} - 0x{:08X})",
        bin.len(),
        min_addr,
        max_addr
    );

    Ok(bin)
}
