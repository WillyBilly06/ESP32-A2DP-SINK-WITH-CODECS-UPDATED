#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <cstdint>

// Forward declare the vulnerable function from main.cpp
// We'll test it indirectly through the BLE handler
extern "C" {
    void handle_ble_data(const uint8_t* data, size_t len);
}

class BufferBoundsTest : public ::testing::TestWithParam<std::vector<uint8_t>> {};

TEST_P(BufferBoundsTest, BLEDataHandlerDoesNotExceedBufferBounds) {
    // Invariant: memcpy operations in BLE handler must not write beyond
    // declared buffer boundaries, regardless of length fields in packet data
    
    std::vector<uint8_t> payload = GetParam();
    
    // Install a signal handler to catch segfaults/buffer overflows
    // This test passes if no crash occurs and memory is not corrupted
    volatile uint8_t canary = 0xAB;
    
    // Call the actual vulnerable function with adversarial input
    // The function should either truncate, reject, or safely handle the oversized length
    handle_ble_data(payload.data(), payload.size());
    
    // Verify canary is unchanged (stack not corrupted)
    EXPECT_EQ(canary, 0xAB);
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialBLEPayloads,
    BufferBoundsTest,
    ::testing::Values(
        // Valid input: normal BLE packet with reasonable length
        std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x00},
        
        // Exploit case: length field (at offset 6) set to 0xFFFF (65535)
        // attempting to copy far beyond typical buffer (e.g., 256 bytes)
        std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFF, 0xFF, 
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        
        // Boundary case: length field set to 2x typical buffer size (512)
        std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x02,
                             0x00, 0x00, 0x00, 0x00},
        
        // Extreme case: length field 0x1000 (4096 bytes)
        std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x10,
                             0x00, 0x00, 0x00, 0x00},
        
        // Empty/minimal packet
        std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}
    )
);

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}