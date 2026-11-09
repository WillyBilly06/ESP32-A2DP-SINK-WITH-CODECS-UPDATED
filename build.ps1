#Requires -Version 7.0
param(
    [Parameter(Position=0)][ValidateSet('build')][string]$Action='build',
    [string]$LhdcSourceDir='',
    [ValidateSet(115200,230400,460800,921600)][int]$ConsoleBaud=921600,
    [switch]$PrintConfiguration
)
$ErrorActionPreference='Stop'
$builder=Join-Path $PSScriptRoot 'native_bt/tools/build_probe.ps1'
if (-not $LhdcSourceDir) {
    $LhdcSourceDir=Join-Path $PSScriptRoot 'native_bt/codecs/lhdcv5/source_snapshot'
}
$LhdcSourceDir=(Resolve-Path -LiteralPath $LhdcSourceDir -ErrorAction Stop).Path
if (-not (Test-Path -LiteralPath (Join-Path $LhdcSourceDir 'decoder/lhdc_dec.c'))) {
    throw 'LHDC source must contain decoder/lhdc_dec.c; no decoder fallback is used'
}
# Default repository build: independent host + Espressif BTDM controller/VHCI.
# Sampling/tick/task profilers stay OFF; this command never opens a device.
$buildOptions=@{
    ConsoleBaud=$ConsoleBaud; ConsoleIrq=$true; StayNative=$true; StorageReserve=$false
    WithProductIo=$true; WithRfcomm=$true
    WithSbc=$true; WithAac=$true; WithAptx=$true; WithLdac=$true; WithOpus=$true; WithLhdc=$true
    AsyncAudio=$true; PcmCopied=$true; PcmTargetMs=16
    BlockDsp=$true; WordDsp=$true; PriorityOrder=$true; CodecAwareRender=$true
    HighRatePcmRebalance=$true; EarlyInputRelease=$true; RenderFairShare=$true
    VolumeRampBlock=$true; DspAssist=$true; DspAssistBlocks=6; DspBlockIram=$true; UnityPcmCopy=$true
    AudioStageProfile=$true; LhdcStageProfile=$true; LhdcSourceDir=$LhdcSourceDir
}
if ($PrintConfiguration) {
    [ordered]@{host='owned';project='native_bt/esp32_probe';hardware_accessed=$false;options=$buildOptions} |
        ConvertTo-Json -Depth 4
    exit 0
}
& $builder @buildOptions
exit $LASTEXITCODE
