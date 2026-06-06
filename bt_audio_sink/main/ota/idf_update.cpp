#include "idf_update.h"

#include <string.h>

#include "esp_log.h"
#include "esp_app_format.h"

#include "mbedtls/md5.h"

static const char *TAG_UPDATE = "IDF_UPDATE";

static int md5_starts(mbedtls_md5_context *ctx) {
    mbedtls_md5_starts(ctx);
    return 0;
}

static int md5_update(mbedtls_md5_context *ctx, const uint8_t *data, size_t len) {
    mbedtls_md5_update(ctx, data, len);
    return 0;
}

static int md5_finish(mbedtls_md5_context *ctx, uint8_t out16[16]) {
    mbedtls_md5_finish(ctx, out16);
    return 0;
}

IdfUpdate::IdfUpdate() {
    reset_();
}

void IdfUpdate::onProgress(ProgressCb cb) {
    _progressCb = cb;
}

bool IdfUpdate::begin(size_t size, const char *label) {
    reset_();

    if (size == 0) {
        abort_(UPDATE_ERROR_BAD_ARGUMENT);
        return false;
    }

    _expectedSize = size;

    const esp_partition_t *part = nullptr;
    if (label && label[0]) {
        part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, label);
        if (!part) {
            ESP_LOGE(TAG_UPDATE, "No app partition with label '%s'", label);
            abort_(UPDATE_ERROR_NO_PARTITION);
            return false;
        }
    } else {
        // Standard 2-partition OTA scheme (ota_0 + ota_1).
        // We have a 3-partition setup (recovery, ota_0, ota_1) but OTA should only
        // alternate between ota_0 and ota_1, never targeting recovery.
        const esp_partition_t *running = esp_ota_get_running_partition();
        
        // Determine the target partition explicitly based on current partition
        if (running) {
            if (strcmp(running->label, "ota_0") == 0) {
                part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
            } else if (strcmp(running->label, "ota_1") == 0) {
                part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
            } else {
                // Running from recovery - target ota_0
                part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
            }
        } else {
            // Fallback: target ota_0
            part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        }
        
        if (!part) {
            ESP_LOGE(TAG_UPDATE, "No OTA partition found for update");
            abort_(UPDATE_ERROR_NO_PARTITION);
            return false;
        }
        
        ESP_LOGI(TAG_UPDATE, "Running from '%s', OTA target is '%s'", 
                 running ? running->label : "unknown", part->label);
    }

    if (_expectedSize > part->size) {
        ESP_LOGE(TAG_UPDATE, "Image too large for slot '%s' (%u > %u)", part->label, (unsigned)_expectedSize, (unsigned)part->size);
        abort_(UPDATE_ERROR_SPACE);
        return false;
    }

    esp_ota_handle_t otaHandle = 0;
    esp_err_t err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
    if (err != ESP_OK) {
        _lastEspErr = err;
        ESP_LOGE(TAG_UPDATE, "esp_ota_begin failed: %s", esp_err_to_name(err));
        abort_(UPDATE_ERROR_WRITE);
        return false;
    }

    _otaHandle = otaHandle;
    _partition = part;

    // MD5 context is always initialized; verification is optional.
    mbedtls_md5_init(&_md5Ctx);
    _md5CtxInit = true;
    (void)md5_starts(&_md5Ctx);

    _running = true;
    ESP_LOGI(TAG_UPDATE, "begin ok slot='%s' addr=0x%08X size=%u", part->label, (unsigned)part->address, (unsigned)_expectedSize);
    return true;
}

bool IdfUpdate::setMD5(const char *expected_md5_hex) {
    if (!expected_md5_hex) {
        abort_(UPDATE_ERROR_BAD_ARGUMENT);
        return false;
    }

    uint8_t tmp[16];
    if (!parse_md5_hex_(expected_md5_hex, tmp)) {
        abort_(UPDATE_ERROR_BAD_ARGUMENT);
        return false;
    }

    memcpy(_md5Expected, tmp, sizeof(_md5Expected));
    _md5ExpectedSet = true;
    _md5Enabled = true;
    return true;
}

size_t IdfUpdate::write(const uint8_t *data, size_t len) {
    if (!data || len == 0) return 0;
    if (!_running || hasError()) return 0;

    // Header verification (first byte written)
    if (!_headerVerified) {
        if (!verifyHeader_(data[0])) {
            return 0;
        }
        _headerVerified = true;
    }

    size_t consumed = 0;
    while (consumed < len) {
        size_t cap = kBlockSize - _bufLen;
        size_t n = (len - consumed < cap) ? (len - consumed) : cap;
        memcpy(&_buf[_bufLen], &data[consumed], n);
        _bufLen += n;
        consumed += n;

        if (_bufLen == kBlockSize) {
            if (!flush_()) {
                break;
            }
        }

        if (_expectedSize && (_progress + _bufLen) > _expectedSize) {
            abort_(UPDATE_ERROR_SIZE);
            break;
        }
    }

    return consumed;
}

bool IdfUpdate::end(bool evenIfRemaining) {
    if (hasError() || !_running) return false;

    if (!flush_()) {
        return false;
    }

    if (!evenIfRemaining && _expectedSize != 0 && _progress != _expectedSize) {
        abort_(UPDATE_ERROR_SIZE);
        return false;
    }

    // MD5 verify (optional)
    if (_md5CtxInit && !_md5Finalized) {
        (void)md5_finish(&_md5Ctx, _md5Actual);
        _md5Finalized = true;
    }

    if (_md5Enabled && _md5ExpectedSet) {
        if (!_md5Finalized) {
            abort_(UPDATE_ERROR_MD5);
            return false;
        }
        if (memcmp(_md5Expected, _md5Actual, sizeof(_md5Expected)) != 0) {
            ESP_LOGE(TAG_UPDATE, "MD5 mismatch");
            abort_(UPDATE_ERROR_MD5);
            return false;
        }
    }

    esp_err_t err = esp_ota_end(_otaHandle);
    if (err != ESP_OK) {
        _lastEspErr = err;
        ESP_LOGE(TAG_UPDATE, "esp_ota_end failed: %s", esp_err_to_name(err));
        abort_(UPDATE_ERROR_WRITE);
        return false;
    }

    err = esp_ota_set_boot_partition(_partition);
    if (err != ESP_OK) {
        _lastEspErr = err;
        ESP_LOGE(TAG_UPDATE, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        abort_(UPDATE_ERROR_ACTIVATE);
        return false;
    }

    _running = false;
    ESP_LOGI(TAG_UPDATE, "end ok");
    return true;
}

void IdfUpdate::abort() {
    abort_(UPDATE_ERROR_ABORT);
}

const char *IdfUpdate::errorString() const {
    switch (_error) {
    case UPDATE_ERROR_OK: return "OK";
    case UPDATE_ERROR_WRITE: return "WRITE";
    case UPDATE_ERROR_ERASE: return "ERASE";
    case UPDATE_ERROR_READ: return "READ";
    case UPDATE_ERROR_SPACE: return "SPACE";
    case UPDATE_ERROR_SIZE: return "SIZE";
    case UPDATE_ERROR_STREAM: return "STREAM";
    case UPDATE_ERROR_MD5: return "MD5";
    case UPDATE_ERROR_MAGIC_BYTE: return "MAGIC_BYTE";
    case UPDATE_ERROR_ACTIVATE: return "ACTIVATE";
    case UPDATE_ERROR_NO_PARTITION: return "NO_PARTITION";
    case UPDATE_ERROR_BAD_ARGUMENT: return "BAD_ARGUMENT";
    case UPDATE_ERROR_ABORT: return "ABORT";
    default: return "UNKNOWN";
    }
}

void IdfUpdate::reset_() {
    _error = UPDATE_ERROR_OK;
    _lastEspErr = ESP_OK;

    _expectedSize = 0;
    _progress = 0;
    _running = false;
    _headerVerified = false;

    _otaHandle = 0;
    _partition = nullptr;

    _bufLen = 0;

    _md5Enabled = false;
    _md5ExpectedSet = false;
    memset(_md5Expected, 0, sizeof(_md5Expected));
    memset(_md5Actual, 0, sizeof(_md5Actual));
    _md5Finalized = false;

    if (_md5CtxInit) {
        mbedtls_md5_free(&_md5Ctx);
        _md5CtxInit = false;
    }
}

void IdfUpdate::abort_(Error err) {
    // Best-effort abort OTA handle if running.
    if (_running && _otaHandle != 0) {
        esp_ota_abort(_otaHandle);
    }

    _error = err;
    _running = false;

    if (_md5CtxInit) {
        mbedtls_md5_free(&_md5Ctx);
        _md5CtxInit = false;
    }

    _otaHandle = 0;
    _partition = nullptr;
    _bufLen = 0;
}

bool IdfUpdate::flush_() {
    if (_bufLen == 0) return true;

    if (_expectedSize && (_progress + _bufLen) > _expectedSize) {
        abort_(UPDATE_ERROR_SIZE);
        return false;
    }

    esp_err_t err = esp_ota_write(_otaHandle, _buf, _bufLen);
    if (err != ESP_OK) {
        _lastEspErr = err;
        ESP_LOGE(TAG_UPDATE, "esp_ota_write failed: %s", esp_err_to_name(err));
        abort_(UPDATE_ERROR_WRITE);
        return false;
    }

    if (_md5CtxInit) {
        (void)md5_update(&_md5Ctx, _buf, _bufLen);
    }

    _progress += _bufLen;
    _bufLen = 0;

    if (_progressCb) {
        _progressCb(_progress, _expectedSize);
    }

    return true;
}

bool IdfUpdate::verifyHeader_(uint8_t firstByte) {
    if (firstByte != ESP_IMAGE_HEADER_MAGIC) {
        ESP_LOGE(TAG_UPDATE, "Invalid magic byte: expected 0x%02X got 0x%02X", (unsigned)ESP_IMAGE_HEADER_MAGIC, (unsigned)firstByte);
        abort_(UPDATE_ERROR_MAGIC_BYTE);
        return false;
    }
    return true;
}

bool IdfUpdate::parse_md5_hex_(const char *hex, uint8_t out16[16]) {
    if (!hex) return false;
    if (strlen(hex) != 32) return false;

    auto nib = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        return -1;
    };

    for (int i = 0; i < 16; i++) {
        int hi = nib(hex[i * 2]);
        int lo = nib(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out16[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}
