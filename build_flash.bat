@echo off
setlocal
set IDF_CCACHE_ENABLE=1
cd /d "%~dp0esp-idf"
call export.bat
if errorlevel 1 exit /b %errorlevel%
cd /d "%~dp0bt_audio_sink"
if "%~1"=="" (
    idf.py --ccache build
) else (
    idf.py --ccache %*
)
exit /b %errorlevel%
