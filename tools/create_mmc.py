#!/usr/bin/env python3
"""Create a blank MMC image file for DivMMC/ESXDOS.

The image contains a FAT16 filesystem with /BIN, /SYS, /TMP directories.

Usage:
    python3 tools/create_mmc.py [output.mmc] [--size SIZE_MB]

Default: esxdos.mmc, 64MB
"""

import argparse
import struct
import os
import sys


def fat16_boot_sector(total_sectors, sectors_per_cluster, reserved_sectors,
                      num_fats, root_entries, sectors_per_fat, volume_label):
    """Build a FAT16 boot sector (512 bytes)."""
    bs = bytearray(512)
    # Jump + NOP
    bs[0:3] = b'\xEB\x3C\x90'
    # OEM name
    bs[3:11] = b'ESXDOS  '
    # Bytes per sector
    struct.pack_into('<H', bs, 11, 512)
    # Sectors per cluster
    bs[13] = sectors_per_cluster
    # Reserved sectors
    struct.pack_into('<H', bs, 14, reserved_sectors)
    # Number of FATs
    bs[16] = num_fats
    # Root directory entries
    struct.pack_into('<H', bs, 17, root_entries)
    # Total sectors (16-bit, 0 if > 65535)
    if total_sectors <= 0xFFFF:
        struct.pack_into('<H', bs, 19, total_sectors)
    else:
        struct.pack_into('<H', bs, 19, 0)
        struct.pack_into('<I', bs, 32, total_sectors)
    # Media type
    bs[21] = 0xF8
    # Sectors per FAT
    struct.pack_into('<H', bs, 22, sectors_per_fat)
    # Sectors per track
    struct.pack_into('<H', bs, 24, 32)
    # Number of heads
    struct.pack_into('<H', bs, 26, 16)
    # Hidden sectors
    struct.pack_into('<I', bs, 28, 0)
    # Drive number
    bs[36] = 0x80
    # Boot signature
    bs[38] = 0x29
    # Volume serial
    struct.pack_into('<I', bs, 39, 0x12345678)
    # Volume label
    bs[43:54] = volume_label.ljust(11).encode('ascii')[:11]
    # Filesystem type
    bs[54:62] = b'FAT16   '
    # Boot signature
    bs[510] = 0x55
    bs[511] = 0xAA
    return bytes(bs)


def fat16_dir_entry(name, attr=0x10, cluster=0, size=0):
    """Create a 32-byte FAT16 directory entry.
    name: 8.3 format, 11 chars padded with spaces.
    attr: 0x10 = directory, 0x20 = archive.
    """
    entry = bytearray(32)
    entry[0:11] = name.ljust(11).encode('ascii')[:11]
    entry[11] = attr
    # Creation time/date
    struct.pack_into('<H', entry, 14, 0x0000)  # time
    struct.pack_into('<H', entry, 16, 0x5421)  # date (2022-01-01)
    struct.pack_into('<H', entry, 18, 0x5421)  # access date
    struct.pack_into('<H', entry, 24, 0x0000)  # modify time
    struct.pack_into('<H', entry, 26, 0x5421)  # modify date
    struct.pack_into('<H', entry, 20, 0)       # cluster high (FAT32 only)
    struct.pack_into('<H', entry, 26, cluster & 0xFFFF)
    struct.pack_into('<I', entry, 28, size)
    return bytes(entry)


def create_mmc(filename, size_mb):
    """Create a blank MMC image with FAT16 filesystem."""
    total_bytes = size_mb * 1024 * 1024
    total_sectors = total_bytes // 512

    # FAT16 parameters
    if size_mb <= 32:
        sectors_per_cluster = 16  # 8KB clusters
    elif size_mb <= 128:
        sectors_per_cluster = 32  # 16KB clusters
    else:
        sectors_per_cluster = 64  # 32KB clusters

    reserved_sectors = 1
    num_fats = 2
    root_entries = 512  # 512 entries = 16 sectors
    root_sectors = (root_entries * 32 + 511) // 512

    # Calculate sectors per FAT
    # data_sectors = total - reserved - root - 2*fat
    # clusters = data_sectors / spc
    # fat_entries = clusters + 2
    # sectors_per_fat = ceil(fat_entries * 2 / 512)
    data_area = total_sectors - reserved_sectors - root_sectors
    # Iterative calculation
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
    # FAT[0] = media byte, FAT[1] = 0xFFFF (end of chain)
    struct.pack_into('<H', fat, 0, 0xFFF8)
    struct.pack_into('<H', fat, 2, 0xFFFF)

    # Allocate clusters for directories: BIN=cluster2, SYS=cluster3, TMP=cluster4
    for i in range(3):
        cluster = 2 + i
        struct.pack_into('<H', fat, cluster * 2, 0xFFFF)  # single-cluster dirs

    # Build root directory
    root = bytearray(root_sectors * 512)
    offset = 0

    # Volume label entry
    vlabel = fat16_dir_entry('ESXDOS     ', attr=0x08, cluster=0, size=0)
    root[offset:offset + 32] = vlabel
    offset += 32

    # BIN directory
    root[offset:offset + 32] = fat16_dir_entry('BIN        ', attr=0x10, cluster=2)
    offset += 32

    # SYS directory
    root[offset:offset + 32] = fat16_dir_entry('SYS        ', attr=0x10, cluster=3)
    offset += 32

    # TMP directory
    root[offset:offset + 32] = fat16_dir_entry('TMP        ', attr=0x10, cluster=4)
    offset += 32

    # Build directory cluster data (. and .. entries)
    cluster_bytes = sectors_per_cluster * 512
    dir_clusters = bytearray(3 * cluster_bytes)
    for i in range(3):
        cluster = 2 + i
        base = i * cluster_bytes
        # . entry
        dir_clusters[base:base + 32] = fat16_dir_entry('.          ', attr=0x10, cluster=cluster)
        # .. entry (root = cluster 0)
        dir_clusters[base + 32:base + 64] = fat16_dir_entry('..         ', attr=0x10, cluster=0)

    # Write image
    data_start = (reserved_sectors + num_fats * sectors_per_fat + root_sectors) * 512
    cluster2_offset = data_start  # cluster 2 starts at data region

    with open(filename, 'wb') as f:
        # Boot sector
        f.write(boot)
        # FAT1
        f.write(fat)
        # FAT2
        f.write(fat)
        # Root directory
        f.write(root)
        # Directory clusters (BIN, SYS, TMP)
        f.write(dir_clusters)
        # Fill rest with zeros
        current = f.tell()
        remaining = total_bytes - current
        if remaining > 0:
            # Write in chunks to avoid memory issues
            chunk = b'\x00' * min(1024 * 1024, remaining)
            while remaining > 0:
                write_size = min(len(chunk), remaining)
                f.write(chunk[:write_size])
                remaining -= write_size

    print(f"Created {filename}: {size_mb}MB FAT16 image")
    print(f"  Sectors: {total_sectors}, Cluster size: {sectors_per_cluster * 512}B")
    print(f"  FAT size: {sectors_per_fat} sectors, Root entries: {root_entries}")
    print(f"  Directories: /BIN, /SYS, /TMP")


def main():
    parser = argparse.ArgumentParser(description='Create blank MMC image for DivMMC/ESXDOS')
    parser.add_argument('output', nargs='?', default='esxdos.mmc',
                        help='Output filename (default: esxdos.mmc)')
    parser.add_argument('--size', type=int, default=64,
                        help='Image size in MB (default: 64)')
    args = parser.parse_args()

    if args.size < 4:
        print("Error: minimum size is 4MB", file=sys.stderr)
        sys.exit(1)
    if args.size > 2048:
        print("Error: maximum size is 2048MB for FAT16", file=sys.stderr)
        sys.exit(1)

    create_mmc(args.output, args.size)


if __name__ == '__main__':
    main()
