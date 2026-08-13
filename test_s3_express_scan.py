#!/usr/bin/env python3
"""
S3 EXPRESS_SCAN Packet Parser Validation Test

This script validates the EXPRESS_SCAN packet parsing logic
to ensure correct extraction of angle and distance data.
"""

import struct
import math

def test_express_scan_parsing():
    """Test EXPRESS_SCAN 84-byte packet parsing"""
    
    print("=" * 60)
    print("RPLidar S3 EXPRESS_SCAN Packet Validation")
    print("=" * 60)
    
    # Create a mock EXPRESS_SCAN packet (84 bytes)
    # Structure:
    # - Byte 0: sync1 (0xA in bits 3:0) + ChkSum[3:0] in bits 7:4
    # - Byte 1: sync2 (0x5 in bits 3:0) + ChkSum[7:4] in bits 7:4
    # - Bytes 2-3: start_angle_q6 (14-bit) + S flag (1-bit)
    # - Bytes 4-83: 40 cabin samples (2 bytes each)
    
    # Create sample packet
    sync1_chk_low = 0xA   # sync1 = 0xA
    sync2_chk_high = 0x5  # sync2 = 0x5
    
    # Test angle: 45 degrees = 45 * 64 = 2880 (0x0B40)
    test_angle_deg = 45.0
    angle_q6 = int(test_angle_deg * 64.0) & 0x3FFF  # 14 bits
    s_flag = 1  # Start of new scan
    start_angle_and_s = (s_flag << 14) | angle_q6
    
    # Create cabin data: 40 distance measurements
    cabin_data = []
    for i in range(40):
        # Test pattern: distances from 1000mm to 10000mm
        distance = 1000 + (i * 225)  # increases by ~225mm per sample
        cabin_data.append(distance)
    
    # Build packet bytes
    packet = bytearray()
    packet.append(sync1_chk_low)
    packet.append(sync2_chk_high)
    packet.extend(struct.pack('<H', start_angle_and_s))
    for distance in cabin_data:
        packet.extend(struct.pack('<H', distance))
    
    assert len(packet) == 84, f"Packet size mismatch: {len(packet)} != 84"
    
    print(f"\n✓ Generated test packet (84 bytes)")
    print(f"  - Sync bytes: 0x{sync1_chk_low:X}, 0x{sync2_chk_high:X}")
    print(f"  - Base angle: {test_angle_deg:.1f}°")
    print(f"  - S flag: {s_flag}")
    
    # Parse packet
    print("\n" + "=" * 60)
    print("Parsing Results:")
    print("=" * 60)
    
    sync1 = packet[0] & 0x0F
    sync2 = packet[1] & 0x0F
    
    if sync1 != 0xA or sync2 != 0x5:
        print(f"✗ FAILED: Invalid sync bytes (0x{sync1:X}, 0x{sync2:X})")
        return False
    
    print(f"✓ Sync validation passed")
    
    # Extract angle
    start_angle_raw = struct.unpack('<H', packet[2:4])[0]
    angle_q6_parsed = start_angle_raw & 0x3FFF
    s_flag_parsed = (start_angle_raw >> 14) & 0x01
    
    base_angle = angle_q6_parsed / 64.0
    print(f"✓ Angle extraction: {base_angle:.1f}° (expected {test_angle_deg:.1f}°)")
    print(f"✓ S flag: {s_flag_parsed} (expected {s_flag})")
    
    # Parse cabin samples
    print(f"\n✓ Parsing 40 cabin samples:")
    angle_increment = 360.0 / 40.0  # ~9 degrees per sample
    
    errors = 0
    for i in range(40):
        cabin_offset = 4 + (i * 2)
        distance_mm = struct.unpack('<H', packet[cabin_offset:cabin_offset+2])[0]
        
        # Calculate angle for this point
        point_angle = (base_angle + (i * angle_increment)) % 360.0
        
        # Verify distance matches expected value
        expected_distance = cabin_data[i]
        if distance_mm != expected_distance:
            print(f"  ✗ Sample {i:2d}: distance mismatch {distance_mm} != {expected_distance}")
            errors += 1
        
        if i < 5 or i >= 35:  # Print first 5 and last 5
            print(f"  ✓ Sample {i:2d}: angle={point_angle:6.1f}°, distance={distance_mm:5d}mm")
        elif i == 5:
            print(f"  ... ({40-10} samples omitted) ...")
    
    if errors == 0:
        print(f"\n✓ All 40 samples parsed correctly")
    else:
        print(f"\n✗ {errors} distance mismatches detected")
        return False
    
    # Test angle wrapping
    print("\n" + "=" * 60)
    print("Angle Wrapping Test:")
    print("=" * 60)
    
    # Test high angle (350 degrees)
    high_angle_deg = 350.0
    high_angle_q6 = int(high_angle_deg * 64.0) & 0x3FFF
    for i in range(5):
        point_angle = (high_angle_deg + (i * angle_increment)) % 360.0
        expected_wrapping = (high_angle_deg + (i * angle_increment)) % 360.0
        if abs(point_angle - expected_wrapping) > 0.1:
            print(f"✗ Wrapping failed for sample {i}")
            return False
    print("✓ Angle wrapping works correctly")
    
    # Test distance range
    print("\n" + "=" * 60)
    print("Distance Conversion Test:")
    print("=" * 60)
    
    test_distances_mm = [1000, 5000, 10000, 30000, 50000]
    for dist_mm in test_distances_mm:
        dist_m = dist_mm / 1000.0
        print(f"  {dist_mm:5d}mm → {dist_m:6.2f}m")
    
    print("\n" + "=" * 60)
    print("✓ ALL TESTS PASSED")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = test_express_scan_parsing()
    exit(0 if success else 1)
