#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "mbedtls/md5.h"

class IdfUpdate {
public:
    enum Error : uint8_t {
        UPDATE_ERROR_OK           = 0,
        UPDATE_ERROR_WRITE        = 1,
        UPDATE_ERROR_ERASE        = 2,
        UPDATE_ERROR_READ         = 3,
        UPDATE_ERROR_SPACE        = 4,
        UPDATE_ERROR_SIZE         = 5,
        UPDATE_ERROR_STREAM       = 6,
        UPDATE_ERROR_MD5          = 7,
        UPDATE_ERROR_MAGIC_BYTE   = 8,
        UPDATE_ERROR_ACTIVATE     = 9,
        UPDATE_ERROR_NO_PARTITION = 10,
        UPDATE_ERROR_BAD_ARGUMENT = 11,
        UPDATE_ERROR_ABORT        = 12,
    };

    using ProgressCb = void (*)(size_t written, size_t total);

    IdfUpdate();

    void onProgress(ProgressCb cb);

    // Starts an OTA update for the next update partition.
    // If label is provided, tries to use that app partition label.
    bool begin(size_t size, const char *label = nullptr);

    // Writes firmware bytes. Returns number of bytes consumed.
    size_t write(const uint8_t *data, size_t len);

    // Finalizes update, validates and sets boot partition.
    bool end(bool evenIfRemaining = false);

    void abort();

    // Optional MD5 verification. Provide hex string (32 chars).
    bool setMD5(const char *expected_md5_hex);

    Error getError() const { return _error; }
    esp_err_t lastEspErr() const { return _lastEspErr; }
    bool hasError() const { return _error != UPDATE_ERROR_OK; }

    bool isRunning() const { return _running; }
    bool isFinished() const { return (_expectedSize > 0) && (_progress == _expectedSize); }

    size_t size() const { return _expectedSize; }
    size_t progress() const { return _progress; }
    size_t remaining() const { return (_expectedSize > _progress) ? (_expectedSize - _progress) : 0; }

    const char *errorString() const;

private:
    static constexpr size_t kBlockSize = 4096;

    void reset_();
    void abort_(Error err);
    bool flush_();
    bool verifyHeader_(uint8_t firstByte);
    bool parse_md5_hex_(const char *hex, uint8_t out16[16]);

private:
    Error _error = UPDATE_ERROR_OK;
    esp_err_t _lastEspErr = ESP_OK;

    ProgressCb _progressCb = nullptr;

    size_t _expectedSize = 0;
    size_t _progress = 0;

    bool _running = false;
    bool _headerVerified = false;

    // OTA
    esp_ota_handle_t _otaHandle = 0;
    const esp_partition_t *_partition = nullptr;

    // Buffer
    uint8_t _buf[kBlockSize];
    size_t _bufLen = 0;

    // MD5
    bool _md5Enabled = false;
    bool _md5ExpectedSet = false;
    uint8_t _md5Expected[16] = {0};
    uint8_t _md5Actual[16] = {0};
    bool _md5Finalized = false;
    mbedtls_md5_context _md5Ctx;
    bool _md5CtxInit = false;
};
