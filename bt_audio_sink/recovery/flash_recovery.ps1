# Flash Recovery Partition
# This script flashes the recovery firmware to the recovery partition (ota_2)
# It does NOT change the OTA boot partition, so normal boot remains on main firmware.

$port = if ($args[0]) { $args[0] } else { "COM3" }

Write-Host "Flashing recovery partition to $port..." -ForegroundColor Cyan

# Flash bootloader, partition table, and recovery firmware
# Recovery goes to 0x10000 (recovery partition, type ota_2)
& python.exe $env:IDF_PATH\components\esptool_py\esptool\esptool.py `
    -p $port -b 460800 --before default_reset --after hard_reset --chip esp32 `
    write_flash --flash_mode dio --flash_size 8MB --flash_freq 40m `
    0x1000 build\bootloader\bootloader.bin `
    0x8000 build\partition_table\partition-table.bin `
    0x10000 build\recovery.bin

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nRecovery partition flashed successfully!" -ForegroundColor Green
    Write-Host "The device will still boot normally (ota_0)." -ForegroundColor Yellow
    Write-Host "To enter recovery mode: hold boot button during power-up." -ForegroundColor Yellow
} else {
    Write-Host "`nFlash failed!" -ForegroundColor Red
}
