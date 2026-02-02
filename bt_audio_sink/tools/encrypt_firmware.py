#!/usr/bin/env python3
"""
ESP32 OTA Firmware Encryption Tool
===================================

This script encrypts ESP32 firmware binaries using AES-256-CBC encryption.
Only the recovery firmware with the matching key can decrypt and flash these files.

Usage:
    python encrypt_firmware.py input.bin --version 1.2.3
    python encrypt_firmware.py --decrypt input.enc output.bin  # For testing

Output files (in ota_releases folder):
    - VERSION.enc (e.g., 1.0.0.enc) - Encrypted firmware
    - latest.txt - Contains "VERSION,FILE_ID" (you fill in FILE_ID after upload)

The encryption key MUST match the AES_KEY in recovery_main.cpp!
"""

import sys
import os
import argparse

try:
    from Crypto.Cipher import AES
    from Crypto.Random import get_random_bytes
    from Crypto.Util.Padding import pad, unpad
except ImportError:
    print("Error: pycryptodome is not installed.")
    print("Install it with: pip install pycryptodome")
    sys.exit(1)

# ============================================================
# IMPORTANT: This key MUST match AES_KEY in recovery_main.cpp!
# Generate your own key with: python encrypt_firmware.py --generate-key
# ============================================================
AES_KEY = bytes([
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
])

BLOCK_SIZE = 16


def encrypt_firmware(input_path: str, output_path: str) -> None:
    """Encrypt a firmware binary file."""
    print(f"Encrypting: {input_path}")
    print(f"Output: {output_path}")
    
    with open(input_path, 'rb') as f:
        plaintext = f.read()
    
    original_size = len(plaintext)
    print(f"Original size: {original_size:,} bytes")
    
    iv = get_random_bytes(BLOCK_SIZE)
    cipher = AES.new(AES_KEY, AES.MODE_CBC, iv)
    padded_data = pad(plaintext, BLOCK_SIZE)
    ciphertext = cipher.encrypt(padded_data)
    
    with open(output_path, 'wb') as f:
        f.write(iv)
        f.write(ciphertext)
    
    encrypted_size = BLOCK_SIZE + len(ciphertext)
    print(f"Encrypted size: {encrypted_size:,} bytes")
    print("✓ Encryption complete!")


def decrypt_firmware(input_path: str, output_path: str) -> None:
    """Decrypt a firmware binary file (for testing)."""
    print(f"Decrypting: {input_path}")
    
    with open(input_path, 'rb') as f:
        data = f.read()
    
    iv = data[:BLOCK_SIZE]
    ciphertext = data[BLOCK_SIZE:]
    
    cipher = AES.new(AES_KEY, AES.MODE_CBC, iv)
    padded_plaintext = cipher.decrypt(ciphertext)
    plaintext = unpad(padded_plaintext, BLOCK_SIZE)
    
    with open(output_path, 'wb') as f:
        f.write(plaintext)
    
    print(f"Decrypted size: {len(plaintext):,} bytes")
    print("✓ Decryption complete!")


def generate_new_key():
    """Generate a new random AES-256 key."""
    new_key = get_random_bytes(32)
    print("Generated new AES-256 key:")
    print()
    print("For Python (encrypt_firmware.py):")
    print(f"AES_KEY = bytes([")
    for i in range(0, 32, 8):
        line = ", ".join(f"0x{b:02X}" for b in new_key[i:i+8])
        print(f"    {line},")
    print("])")
    print()
    print("For C++ (recovery_main.cpp):")
    print("static const uint8_t AES_KEY[32] = {")
    for i in range(0, 32, 8):
        line = ", ".join(f"0x{b:02X}" for b in new_key[i:i+8])
        comma = "," if i < 24 else ""
        print(f"    {line}{comma}")
    print("};")


def main():
    parser = argparse.ArgumentParser(description="Encrypt ESP32 OTA firmware")
    parser.add_argument('input', nargs='?', help='Input firmware binary file')
    parser.add_argument('--version', '-v', help='Firmware version (e.g., 1.0.0)')
    parser.add_argument('--output-dir', default='ota_releases', help='Output directory')
    parser.add_argument('--decrypt', '-d', action='store_true', help='Decrypt mode')
    parser.add_argument('--output', '-o', help='Output file (for decrypt)')
    parser.add_argument('--generate-key', '-g', action='store_true', help='Generate new key')
    
    args = parser.parse_args()
    
    if args.generate_key:
        generate_new_key()
        return
    
    if args.decrypt:
        if not args.input:
            print("Error: Input file required")
            sys.exit(1)
        output = args.output or args.input.replace('.enc', '_decrypted.bin')
        decrypt_firmware(args.input, output)
        return
    
    if not args.input:
        parser.print_help()
        sys.exit(1)
    
    if not args.version:
        print("Error: --version is required")
        print("Example: python encrypt_firmware.py firmware.bin --version 1.0.0")
        sys.exit(1)
    
    if not os.path.exists(args.input):
        print(f"Error: Input file not found: {args.input}")
        sys.exit(1)
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Output filename is just VERSION.enc
    output_filename = f"{args.version}.enc"
    output_path = os.path.join(args.output_dir, output_filename)
    
    # Encrypt
    encrypt_firmware(args.input, output_path)
    
    # Create latest.txt template
    latest_path = os.path.join(args.output_dir, "latest.txt")
    with open(latest_path, 'w') as f:
        f.write(f"{args.version},PUT_FIRMWARE_FILE_ID_HERE\n")
    
    print()
    print("=" * 60)
    print("FILES CREATED:")
    print("=" * 60)
    print(f"  1. {output_path}")
    print(f"  2. {latest_path}")
    print()
    print("=" * 60)
    print("NEXT STEPS:")
    print("=" * 60)
    print()
    print(f"1. Upload {output_filename} to your Google Drive folder")
    print("   - Right-click → Share → Anyone with link")
    print("   - Copy the FILE_ID from the URL")
    print()
    print("2. Edit latest.txt and replace PUT_FIRMWARE_FILE_ID_HERE")
    print(f"   with the actual FILE_ID of {output_filename}")
    print()
    print("3. Upload latest.txt to the same Google Drive folder")
    print("   - Right-click → Share → Anyone with link")
    print("   - Copy the FILE_ID of latest.txt")
    print()
    print("4. Put the latest.txt FILE_ID in recovery_main.cpp:")
    print(f'   GDRIVE_LATEST_TXT_ID = "FILE_ID_OF_LATEST_TXT"')
    print()


if __name__ == '__main__':
    main()
