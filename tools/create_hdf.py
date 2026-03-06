#!/usr/bin/env python3
"""Create a blank HDF image file for DivIDE/ESXDOS.

The image contains an RS-IDE header with ATA IDENTIFY data and a FAT16
filesystem with /BIN, /SYS, /TMP directories.

Usage:
    python3 tools/create_hdf.py [output.hdf] [--size SIZE_MB]

Default: esxdos.hdf, 64MB
"""

import argparse
import struct
import sys


def ata_identify(cylinders, heads, sectors_per_track, total_sectors, model):
    """Build a 512-byte ATA IDENTIFY DEVICE response."""
    ident = bytearray(512)
    # Word 0: General config (fixed disk)
    struct.pack_into('<H', ident, 0, 0x0040)
    # Word 1: Cylinders
    struct.pack_into('<H', ident, 2, cylinders)
    # Word 3: Heads
    struct.pack_into('<H', ident, 6, heads)
    # Word 6: Sectors per track
    struct.pack_into('<H', ident, 12, sectors_per_track)
    # Words 10-19: Serial number (20 chars, space-padded)
    serial = 'ESXDOS0001'.ljust(20).encode('ascii')[:20]
    # ATA strings are byte-swapped (high byte first in each word)
    for i in range(10):
        ident[20 + i * 2] = serial[i * 2 + 1]
        ident[20 + i * 2 + 1] = serial[i * 2]
    # Words 23-26: Firmware revision (8 chars)
    fw = 'v1.0'.ljust(8).encode('ascii')[:8]
    for i in range(4):
        ident[46 + i * 2] = fw[i * 2 + 1]
        ident[46 + i * 2 + 1] = fw[i * 2]
    # Words 27-46: Model (40 chars)
    mdl = model.ljust(40).encode('ascii')[:40]
    for i in range(20):
        ident[54 + i * 2] = mdl[i * 2 + 1]
        ident[54 + i * 2 + 1] = mdl[i * 2]
    # Word 47: Max sectors per interrupt (R/W multiple)
    struct.pack_into('<H', ident, 94, 0x0001)
    # Word 49: Capabilities (LBA supported)
    struct.pack_into('<H', ident, 98, 0x0200)
    # Word 60-61: Total addressable sectors (LBA)
    struct.pack_into('<I', ident, 120, total_sectors)
    return bytes(ident)


def fat16_boot_sector(total_sectors, sectors_per_cluster, reserved_sectors,
                      num_fats, root_entries, sectors_per_fat, volume_label):
    """Build a FAT16 boot sector (512 bytes)."""
    bs = bytearray(512)
    bs[0:3] = b'\xEB\x3C\x90'
    bs[3:11] = b'ESXDOS  '
    struct.pack_into('<H', bs, 11, 512)
    bs[13] = sectors_per_cluster
    struct.pack_into('<H', bs, 14, reserved_sectors)
    bs[16] = num_fats
    struct.pack_into('<H', bs, 17, root_entries)
    if total_sectors <= 0xFFFF:
        struct.pack_into('<H', bs, 19, total_sectors)
    else:
        struct.pack_into('<H', bs, 19, 0)
        struct.pack_into('<I', bs, 32, total_sectors)
    bs[21] = 0xF8
    struct.pack_into('<H', bs, 22, sectors_per_fat)
    struct.pack_into('<H', bs, 24, 32)
    struct.pack_into('<H', bs, 26, 16)
    struct.pack_into('<I', bs, 28, 0)
    bs[36] = 0x80
    bs[38] = 0x29
    struct.pack_into('<I', bs, 39, 0x12345678)
    bs[43:54] = volume_label.ljust(11).encode('ascii')[:11]
    bs[54:62] = b'FAT16   '
    bs[510] = 0x55
    bs[511] = 0xAA
    return bytes(bs)


def fat16_dir_entry(name, attr=0x10, cluster=0, size=0):
    """Create a 32-byte FAT16 directory entry."""
    entry = bytearray(32)
    entry[0:11] = name.ljust(11).encode('ascii')[:11]
    entry[11] = attr
    struct.pack_into('<H', entry, 14, 0x0000)
    struct.pack_into('<H', entry, 16, 0x5421)
    struct.pack_into('<H', entry, 18, 0x5421)
    struct.pack_into('<H', entry, 24, 0x0000)
    struct.pack_into('<H', entry, 26, 0x5421)
    struct.pack_into('<H', entry, 20, 0)
    struct.pack_into('<H', entry, 26, cluster & 0xFFFF)
    struct.pack_into('<I', entry, 28, size)
    return bytes(entry)


def create_hdf(filename, size_mb):
    """Create a blank HDF image with RS-IDE header and FAT16 filesystem."""
    # HDF header is 128 bytes
    # Data offset = 128 + ATA IDENTIFY (512) = 0x280 = 640
    # But standard HDF: header=128 bytes, IDENTIFY at 0x16 (22) within header (106 bytes used)
    # Data starts at offset stored in header bytes 9-10

    # RS-IDE HDF v1.1 format:
    # 0x00-0x06: "RS-IDE" + 0x1A
    # 0x07: HDF version (0x11 = v1.1)
    # 0x08: flags (bit0: LBA28)
    # 0x09-0x0A: data offset (little-endian, in bytes)
    # 0x0B-0x15: reserved
    # 0x16-0x7F: ATA IDENTIFY data (first 106 bytes of the 512-byte response)

    header = bytearray(128)
    header[0:6] = b'RS-IDE'
    header[6] = 0x1A
    header[7] = 0x11  # v1.1
    header[8] = 0x01  # LBA28 mode

    # Data offset = 128 + 384 (padding to align to 512) = 512
    # Actually, simplest: data starts right after header at 128
    # But many implementations use sector-aligned data
    # Standard: data_offset = 128 (header size)
    data_offset = 128
    struct.pack_into('<H', header, 9, data_offset)

    # CHS geometry
    total_bytes = size_mb * 1024 * 1024
    total_sectors = total_bytes // 512
    heads = 16
    sectors_per_track = 63
    cylinders = total_sectors // (heads * sectors_per_track)
    if cylinders > 16383:
        cylinders = 16383

    # ATA IDENTIFY (only first 106 bytes stored in header at 0x16)
    ident_full = ata_identify(cylinders, heads, sectors_per_track,
                              total_sectors, 'ESXDOS HDF Image')
    header[0x16:0x16 + 106] = ident_full[:106]

    # FAT16 parameters
    if size_mb <= 32:
        sectors_per_cluster = 16
    elif size_mb <= 128:
        sectors_per_cluster = 32
    else:
        sectors_per_cluster = 64

    reserved_sectors = 1
    num_fats = 2
    root_entries = 512
    root_sectors = (root_entries * 32 + 511) // 512

    # Calculate sectors per FAT
    data_area = total_sectors - reserved_sectors - root_sectors
    spf = 1
    for _ in range(20):
        data_sectors = data_area - num_fats * spf
        clusters = data_sectors // sectors_per_cluster
        spf_new = (clusters * 2 + 512 + 511) // 512
        if spf_new <= spf:
            break
        spf = spf_new
    sectors_per_fat = spf

    # Build boot sector
    boot = fat16_boot_sector(total_sectors, sectors_per_cluster,
                             reserved_sectors, num_fats, root_entries,
                             sectors_per_fat, 'ESXDOS')

    # Build FAT
    fat = bytearray(sectors_per_fat * 512)
    struct.pack_into('<H', fat, 0, 0xFFF8)
    struct.pack_into('<H', fat, 2, 0xFFFF)
    for i in range(3):
        cluster = 2 + i
        struct.pack_into('<H', fat, cluster * 2, 0xFFFF)

    # Build root directory
    root = bytearray(root_sectors * 512)
    offset = 0
    vlabel = fat16_dir_entry('ESXDOS     ', attr=0x08, cluster=0, size=0)
    root[offset:offset + 32] = vlabel
    offset += 32
    root[offset:offset + 32] = fat16_dir_entry('BIN        ', attr=0x10, cluster=2)
    offset += 32
    root[offset:offset + 32] = fat16_dir_entry('SYS        ', attr=0x10, cluster=3)
    offset += 32
    root[offset:offset + 32] = fat16_dir_entry('TMP        ', attr=0x10, cluster=4)
    offset += 32

    # Build directory cluster data (. and .. entries)
    cluster_bytes = sectors_per_cluster * 512
    dir_clusters = bytearray(3 * cluster_bytes)
    for i in range(3):
        cluster = 2 + i
        base = i * cluster_bytes
        dir_clusters[base:base + 32] = fat16_dir_entry('.          ', attr=0x10, cluster=cluster)
        dir_clusters[base + 32:base + 64] = fat16_dir_entry('..         ', attr=0x10, cluster=0)

    # Write image
    with open(filename, 'wb') as f:
        # HDF header (128 bytes)
        f.write(bytes(header))
        # FAT16 boot sector
        f.write(boot)
        # FAT1
        f.write(fat)
        # FAT2
        f.write(fat)
        # Root directory
        f.write(root)
        # Directory clusters
        f.write(dir_clusters)
        # Fill rest
        current = f.tell()
        target = data_offset + total_bytes
        remaining = target - current
        if remaining > 0:
            chunk = b'\x00' * min(1024 * 1024, remaining)
            while remaining > 0:
                write_size = min(len(chunk), remaining)
                f.write(chunk[:write_size])
                remaining -= write_size

    print(f"Created {filename}: {size_mb}MB HDF image")
    print(f"  CHS: {cylinders}/{heads}/{sectors_per_track}, LBA sectors: {total_sectors}")
    print(f"  Data offset: {data_offset}")
    print(f"  FAT16: cluster size {sectors_per_cluster * 512}B, FAT {sectors_per_fat} sectors")
    print(f"  Directories: /BIN, /SYS, /TMP")


def main():
    parser = argparse.ArgumentParser(description='Create blank HDF image for DivIDE/ESXDOS')
    parser.add_argument('output', nargs='?', default='esxdos.hdf',
                        help='Output filename (default: esxdos.hdf)')
    parser.add_argument('--size', type=int, default=64,
                        help='Image size in MB (default: 64)')
    args = parser.parse_args()

    if args.size < 4:
        print("Error: minimum size is 4MB", file=sys.stderr)
        sys.exit(1)
    if args.size > 2048:
        print("Error: maximum size is 2048MB for FAT16", file=sys.stderr)
        sys.exit(1)

    create_hdf(args.output, args.size)


if __name__ == '__main__':
    main()
