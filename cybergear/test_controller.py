from cybergear.controller import *
import pytest

def test_upack_AT():
    raw = bytes.fromhex('41 54 90 07 eb fc 08 05 70 00 00 07 01 95 54 0d 0a')
    data = YourCeeBridge.unpack_AT(raw)
    assert data.mode == 18
    assert data.id == 127
    assert data.data2 == 0x00fd
    assert data.data1 == bytes.fromhex('05 70 00 00 07 01 95 54')


def test_pack_AT():
    raw = MiData(18, 0x00fd, 127, bytes.fromhex('05 70 00 00 07 01 95 54'))
    packed = YourCeeBridge.pack_AT(raw)
    assert packed == bytes.fromhex('41 54 90 07 eb fc 08 05 70 00 00 07 01 95 54 0d 0a')

def test_Req1():
    assert (Req1(1, 0, 0, 0, 0, 0).data1 == 
           bytes.fromhex('ff 7f ff 7f 00 00 00 00'))
    assert (Req1(1, -1000, -1000, -1000, 1, 0).data1 == 
           ## 83 00 小端
           bytes.fromhex('00 00 00 00 83 00 00 00'))

def test_extract_bits():
    assert extract_bits(0x12345678, 8, 15) == 0x56
    assert extract_bits(0x12345678, 16, 23) == 0x34
    assert extract_bits(0x12345678, 24, 31) == 0x12
    assert extract_bits(0x12345678, 0, 7) == 0x78
    assert extract_bits(0b11110000, 4, 4) == 0b1
    assert extract_bits(0b11110000, 3, 3) == 0b0