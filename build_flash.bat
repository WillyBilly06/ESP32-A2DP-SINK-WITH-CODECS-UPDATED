@echo off
cd /d "D:\Bluetooth Project\esp-idf"
call export.bat
cd /d "D:\Bluetooth Project\bt_audio_sink"
idf.py flash -p COM3 monitor
pause
