import argparse
import struct

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30

parser = argparse.ArgumentParser()
parser.add_argument("blit_file", help=".blit to append files to")
parser.add_argument("uf2_file", help=".blit to append files to")
parser.add_argument("output_file")

args = parser.parse_args()

pad_uf2_size = 0 #8 * 1024 * 1024 # only need this if uf2 accesses flash outside itself (like a filesystem)

blit_in = open(args.blit_file, 'rb')
blit_out = open(args.output_file, 'wb')
uf2_in = open(args.uf2_file, 'rb')

# get blit end offset
head = blit_in.read(20)

blit_end, = struct.unpack('<I', head[16:20])

print(f'blit end {blit_end}')

blit_in.seek(0)
blit_out.write(blit_in.read(blit_end))

# pad to 4k
while blit_out.tell() & 0xFFF:
    blit_out.write(b'\xff')

# copy uf2 data
uf2_out_offset = blit_out.tell()
uf2_end_addr = 0
print(f'padded to {uf2_out_offset}')

while True:
    block = uf2_in.read(512)
    if len(block) == 0:
        break

    magic_start0, magic_start1, flags, addr, payload_size, block_no, num_blocks, family_id_size, data, magic_end = struct.unpack("<IIIIIIII476sI", block)
    if magic_start0 != UF2_MAGIC_START0 or magic_start1 != UF2_MAGIC_START1 or magic_end != UF2_MAGIC_END:
        print('bad uf2 magic')
        exit(1)

    # skip the E10 block
    if block_no == 0 and num_blocks == 2 and addr == 0x10FFFF00:
        continue

    assert addr >> 28 == 1

    if addr + payload_size > uf2_end_addr:
        uf2_end_addr = addr + payload_size

    blit_out.seek(uf2_out_offset + addr - 0x10000000)
    blit_out.write(data[:payload_size])

uf2_data_len = uf2_end_addr - 0x10000000
print(f'uf2 data len {uf2_data_len}')

# pad to pad_uf2_size
if pad_uf2_size:
    blit_out.seek(uf2_out_offset + uf2_data_len)
    pad_size = pad_uf2_size - uf2_data_len
    print(f'add {pad_size} padding bytes')
    blit_out.write(b'\xff' * pad_size)

# update header
new_end = blit_out.tell()
print(f'new end {new_end}')
blit_out.seek(16)
blit_out.write(struct.pack("<I", new_end))

# copy metadata?
blit_out.seek(new_end)
meta = blit_in.read()
print(f'metadata len {len(meta)}')
blit_out.write(meta)
