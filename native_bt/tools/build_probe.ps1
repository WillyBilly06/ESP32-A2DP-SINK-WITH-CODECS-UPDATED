param([ValidateSet(115200,230400,460800,921600)][int]$ConsoleBaud=921600,[switch]$ConsoleIrq,[switch]$HeapAllocTrace,[switch]$DispatchTrace,[switch]$TaskRuntimeStats,[switch]$TaskRuntimeOnly,[switch]$TaskCycleAccounting,[switch]$HighRatePcmRebalance,[switch]$EarlyInputRelease,[switch]$MatchedFeedback,[switch]$MatchedFeedbackCompare,[switch]$RenderFairShare,[switch]$VolumeRampBlock,[switch]$VolumeRampCompare,[switch]$OwnerPacer,[switch]$DspAssist,[ValidateSet(4,5,6)][int]$DspAssistBlocks=4,[switch]$DspBlockIram,[switch]$UnityPcmCopy,[switch]$LhdcCore0Render,[switch]$LhdcFastMantissa,[switch]$AptxSnapshot,[switch]$StayNative,[switch]$StorageReserve,[switch]$WordDsp,[switch]$PriorityOrder,[ValidateSet(5,6,21)][int]$DecoderPriority=5,[switch]$SchedulerTrace,[switch]$CodecAwareRender,[switch]$PcmCopied,[switch]$PcmHash,[ValidateSet(-1,0,1)][int]$LoggerCore=-1,[switch]$BlockDsp,[ValidateSet(16,32,64)][int]$PcmTargetMs=64,[switch]$PcmLoans,[switch]$DspInlineCheck,[ValidateSet(0,1)][int]$DecoderCore=0,[switch]$RfProtocolDiagnostic,[switch]$FirCompilerCheck,[switch]$DspStageProfile,[switch]$AudioStageProfile,[switch]$PipelineDiagnostic,[switch]$AsyncAudio,[switch]$DeferredCodecCheck,[switch]$LhdcStageProfile,[switch]$LhdcMaximumCheck,[switch]$CodecDiagnosticOnly,[switch]$WithProductIo,[switch]$WithProductSecurity,[switch]$WithRfcomm,[switch]$WithLhdc,[switch]$WithOpus,[switch]$WithAac,[switch]$WithAptx,[switch]$WithLdac,[switch]$WithSmp,[switch]$WithAvrcp,[switch]$WithSbc,[switch]$WithAudioProbe,[switch]$WithAudioOutput,[switch]$WithGatt,[switch]$WithClassicSecurity,[switch]$TraceIngress,[switch]$WithoutRxFlow,[switch]$KeepOwned=$true,[string]$TestPeer='',[string]$LhdcSourceDir='')
$ErrorActionPreference = 'Stop'
if ($StorageReserve -and (-not $StayNative -or -not $AsyncAudio -or -not $PcmCopied)) { throw 'Experimental storage reserve requires native async copied PCM' }
if ($HeapAllocTrace -and (-not $StayNative -or -not $AsyncAudio -or -not $ConsoleIrq -or $SchedulerTrace -or $TaskRuntimeStats -or $TaskRuntimeOnly -or $TaskCycleAccounting -or $DispatchTrace)) { throw 'Allocation sampling requires full native IRQ audio without other task profilers' }
if ($TaskRuntimeOnly) {
    if ($TaskCycleAccounting) { throw 'Runtime-only observation excludes task-cycle accounting' }
    if ($SchedulerTrace) { throw 'Runtime-only observation excludes detailed scheduler tracing' }
    $TaskRuntimeStats=$true
}
if ($DispatchTrace -and (-not $StayNative -or -not $AsyncAudio -or -not $PcmCopied -or -not $DspAssist -or -not $WithLhdc -or $SchedulerTrace -or $TaskRuntimeStats -or $TaskCycleAccounting)) { throw 'Frozen dispatch trace requires native copied LHDC/helper output without other task profilers' }
if ($DspAssistBlocks -ne 4 -and -not $DspAssist) { throw 'Nondefault DSP batch requires -DspAssist' }
if ($TaskCycleAccounting -and -not $TaskRuntimeStats) { throw 'Task-cycle accounting requires task runtime statistics' }
if ($ConsoleIrq -and (-not $StayNative -or -not $AsyncAudio)) { throw 'Console IRQ requires complete native asynchronous output' }
if ($TaskRuntimeStats -and (-not $StayNative -or (-not $SchedulerTrace -and -not $TaskRuntimeOnly) -or -not $AsyncAudio)) { throw 'Task runtime requires complete native sampled asynchronous output' }
if ($HighRatePcmRebalance -and (-not $EarlyInputRelease -or -not $WithLhdc -or $PcmTargetMs -ne 16 -or $LhdcCore0Render)) { throw 'High-rate PCM rebalance requires Core1 LHDC early release and configured16ms reserve' }
if ($EarlyInputRelease -and (-not $StayNative -or -not $CodecAwareRender -or -not $AsyncAudio -or -not $PcmCopied -or -not $UnityPcmCopy -or -not $BlockDsp -or $MatchedFeedbackCompare -or $VolumeRampCompare -or $OwnerPacer)) { throw 'Early input release requires full native copied/unity/block output without kernel comparisons or pacing' }
if ($MatchedFeedbackCompare) { $MatchedFeedback=$true }
if ($MatchedFeedback -and (-not $StayNative -or -not $BlockDsp -or -not $WordDsp)) { throw 'Matched feedback requires native word/block DSP' }
if ($MatchedFeedbackCompare -and ($VolumeRampCompare -or $OwnerPacer)) { throw 'Matched comparison excludes volume comparison/owner pacing' }
if ($PriorityOrder) { $DecoderPriority=6; $KeepOwned=$false }
if ($AptxSnapshot -and (-not $StayNative -or -not $WithAptx -or -not $PcmCopied -or -not $BlockDsp -or -not $WordDsp -or -not $CodecAwareRender)) { throw 'aptX capture requires complete stay-native aptX copied PCM, block/word DSP and codec-aware rendering' }
if ($RenderFairShare -and (-not $StayNative -or -not $PriorityOrder -or -not $CodecAwareRender -or -not $AsyncAudio -or -not $PcmCopied -or $OwnerPacer)) { throw 'Core1 fair sharing requires complete native ordered output, excluding owner pacing' }
if ($VolumeRampBlock -and (-not $StayNative -or -not $BlockDsp)) { throw 'Volume-ramp block requires full native block DSP' }
if ($VolumeRampCompare -and -not $VolumeRampBlock) { throw 'Volume-ramp comparison requires -VolumeRampBlock' }
if ($OwnerPacer -and (-not $StayNative -or -not $PriorityOrder -or -not $AsyncAudio -or -not $PcmCopied -or -not $CodecAwareRender)) { throw 'Host CPU budget requires native priority-ordered asynchronous copied codec-aware audio' }
if ($DspAssist -and (-not $StayNative -or -not $CodecAwareRender -or -not $AsyncAudio -or -not $PcmCopied -or -not $BlockDsp -or -not $UnityPcmCopy -or -not $WithLhdc -or $LhdcCore0Render -or $AptxSnapshot)) { throw 'DSP assistance requires full native LHDC copied/block/unity output on Core1' }
if ($DspBlockIram -and -not $BlockDsp) { throw 'Block DSP IRAM placement requires -BlockDsp' }
if ($UnityPcmCopy -and (-not $StayNative -or -not $PcmCopied -or -not $BlockDsp)) { throw 'Unity PCM copy requires -StayNative -PcmCopied -BlockDsp' }
if ($LhdcCore0Render -and (-not $WithLhdc -or -not $CodecAwareRender)) { throw 'LHDC Core0 rendering requires -WithLhdc -CodecAwareRender' }
if ($LhdcFastMantissa -and -not $WithLhdc) { throw 'LHDC mantissa candidate requires -WithLhdc' }
if ($WordDsp -and -not $BlockDsp) { throw 'Word DSP requires -BlockDsp' }
if ($LoggerCore -lt 0) { $LoggerCore=1-$DecoderCore }
if ($PcmCopied -and $PcmLoans) { throw 'PCM input modes are mutually exclusive' }
if ($DspInlineCheck -and (-not $AudioStageProfile -or $DspStageProfile -or $FirCompilerCheck)) { throw 'DSP inline comparison requires -AudioStageProfile and excludes other kernel comparisons' }
if ($RfProtocolDiagnostic) {
    if (-not $WithProductIo -or $AsyncAudio -or $CodecDiagnosticOnly -or $PipelineDiagnostic) { throw 'RF protocol diagnostic requires -WithProductIo, excludes async output/other diagnostics, and never keeps the image active' }
    $KeepOwned=$false
}
if ($FirCompilerCheck -and (-not $AudioStageProfile -or $DspStageProfile)) { throw 'FIR compiler check requires -AudioStageProfile and excludes -DspStageProfile' }
if ($DspStageProfile -and -not $AudioStageProfile) { throw 'DSP stage profiling requires -AudioStageProfile' }
if ($AudioStageProfile -and -not $PipelineDiagnostic -and -not $StayNative) { throw 'Audio stage profiling requires -PipelineDiagnostic or the complete -StayNative product' }
if ($PipelineDiagnostic) {
    if ($DeferredCodecCheck -or $LhdcMaximumCheck -or $LhdcStageProfile) { throw 'Pipeline diagnostic is a separate finite check' }
    $CodecDiagnosticOnly=$true; $WithLhdc=$true; $WithAudioOutput=$true; $AsyncAudio=$true; $KeepOwned=$false
}
if ($DeferredCodecCheck -and -not $CodecDiagnosticOnly) { throw 'Deferred codec check requires -CodecDiagnosticOnly' }
if ($LhdcStageProfile -and -not $LhdcMaximumCheck -and -not ($StayNative -and $WithLhdc -and $WithProductIo -and $AsyncAudio -and $AudioStageProfile -and -not $CodecDiagnosticOnly -and -not $PipelineDiagnostic)) { throw 'LHDC stage profiling requires the maximum diagnostic or complete live native audio with -AudioStageProfile' }
if ($LhdcMaximumCheck -and (-not $CodecDiagnosticOnly -or -not $WithLhdc)) { throw 'Maximum LHDC check requires -CodecDiagnosticOnly -WithLhdc' }
if ($CodecDiagnosticOnly) {
    if ($WithProductIo -or $WithProductSecurity -or $WithRfcomm -or $WithSmp -or $WithAvrcp -or $WithAudioProbe -or ($WithAudioOutput -and -not $PipelineDiagnostic) -or $WithGatt -or $WithClassicSecurity -or $TraceIngress) { throw 'Codec-only diagnostic excludes radio and product features' }
    $KeepOwned=$false
    $WithSbc=$true
}
if ($WithProductIo) { $WithProductSecurity=$true; $WithSmp=$true; $WithGatt=$true; $WithAudioOutput=$true; $WithAvrcp=$true; $WithClassicSecurity=$true }
if ($WithRfcomm) { $WithClassicSecurity=$true }
if ($WithAudioOutput -and -not $PipelineDiagnostic) { $WithAudioProbe = $true }
if ($AsyncAudio -and -not $WithAudioOutput) { throw 'Async audio requires -WithAudioOutput or -WithProductIo' }
if ($SchedulerTrace) {
    if (-not $WithAudioProbe -or $CodecDiagnosticOnly -or $PipelineDiagnostic) { throw 'Scheduler trace requires live audio and excludes synthetic diagnostics' }
    $KeepOwned=$false
}
if ($PcmLoans) {
    if (-not $AsyncAudio -or -not $WithAudioOutput -or $RfProtocolDiagnostic) { throw 'PCM loans require async I2S and exclude checksum-only RF mode' }
    $KeepOwned=$false
}
if ($PcmCopied) {
    if (-not $AsyncAudio -or -not $WithAudioOutput -or $RfProtocolDiagnostic) { throw 'Compact copied PCM requires async I2S and excludes checksum-only RF' }
    $KeepOwned=$false
}
if ($DecoderPriority -ne 5) {
    if (-not $WithAudioProbe -or -not $AsyncAudio -or $RfProtocolDiagnostic -or $CodecDiagnosticOnly -or $PipelineDiagnostic) { throw 'Decoder priority experiment requires finite live async audio' }
    $KeepOwned=$false
}
if ($PcmTargetMs -ne 64 -and -not ($PcmLoans -or $PcmCopied)) { throw 'Short PCM target requires a finite async PCM input experiment' }
if ($BlockDsp -and -not ($PcmLoans -or $PcmCopied)) { throw 'Block DSP requires a finite async PCM input experiment' }
if ($WithAudioProbe -and -not $WithClassicSecurity) { throw 'The SBC diagnostic requires -WithClassicSecurity and the exact test peer' }
if ($WithAudioProbe) { $WithSbc = $true }
if ($TraceIngress -and -not $WithClassicSecurity) {
    throw 'The ingress trace requires -WithClassicSecurity and its restricted test peer'
}
if (($TestPeer -and $TestPeer -notmatch '^[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}$') -or ($WithClassicSecurity -and -not $WithProductSecurity -and -not $TestPeer)) {
    throw 'The Classic pairing probe requires -TestPeer with the exact test peer address'
}
if ($StayNative) {
    if (-not $WithProductIo -or -not $AsyncAudio -or $CodecDiagnosticOnly -or $PipelineDiagnostic -or $RfProtocolDiagnostic -or $DspStageProfile -or $FirCompilerCheck -or $DspInlineCheck) { throw 'Stay-native requires the complete asynchronous product entry' }
    if ($DecoderCore -ne 0 -or $LoggerCore -ne 1) { throw 'Stay-native product reserves CPU0 for decoding; use -DecoderCore 0 -LoggerCore 1' }
    $DecoderPriority=21
    $KeepOwned=$true
}
if ($CodecAwareRender -and (-not $StayNative -or -not $AsyncAudio -or -not $PcmCopied -or $PcmLoans)) { throw 'Codec-aware render requires complete stay-native async copied PCM' }
$nativeRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $nativeRoot
$idfRoot = Join-Path $workspaceRoot 'esp-idf'
$buildRoot = Join-Path $nativeRoot '_build\esp32'
$probeRoot = Join-Path $nativeRoot 'esp32_probe'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
if (-not (Test-Path -LiteralPath (Join-Path $probeRoot 'sdkconfig'))) { Copy-Item -LiteralPath (Join-Path $probeRoot 'sdkconfig.defaults') -Destination (Join-Path $probeRoot 'sdkconfig') }
$buildReceiptPath=Join-Path $buildRoot 'local-build-status.json'
@{schema=1;passed=$false;state='building';hardware_accessed=$false} | ConvertTo-Json | Set-Content -LiteralPath $buildReceiptPath -Encoding utf8
$consoleSdkPath=Join-Path $probeRoot 'sdkconfig'
if (Test-Path -LiteralPath $consoleSdkPath) {
    $consoleSdk=[IO.File]::ReadAllText($consoleSdkPath)
    # Custom UART0 keeps GPIO1/3 but makes the baud Kconfig prompt visible.
    # Remove generated legacy aliases too, so they cannot restore115200.
    $consoleKeys='(?:ESP_)?CONSOLE_UART_(?:DEFAULT|CUSTOM|CUSTOM_NUM_0|CUSTOM_NUM_1|NUM|TX_GPIO|RX_GPIO|BAUDRATE)'
    $consoleSdk=[regex]::Replace($consoleSdk,'(?m)^(?:# )?CONFIG_'+$consoleKeys+'(?:=.*| is not set)\r?$','')
    $consoleSdk += "`n# CONFIG_ESP_CONSOLE_UART_DEFAULT is not set`nCONFIG_ESP_CONSOLE_UART_CUSTOM=y`nCONFIG_ESP_CONSOLE_UART_CUSTOM_NUM_0=y`n# CONFIG_ESP_CONSOLE_UART_CUSTOM_NUM_1 is not set`nCONFIG_ESP_CONSOLE_UART_TX_GPIO=1`nCONFIG_ESP_CONSOLE_UART_RX_GPIO=3`nCONFIG_ESP_CONSOLE_UART_BAUDRATE=$ConsoleBaud`n"
    [IO.File]::WriteAllText($consoleSdkPath,$consoleSdk,[Text.UTF8Encoding]::new($false))
}
if ($WithAudioOutput) {
    $nativeSdkPath=Join-Path $probeRoot 'sdkconfig'
    if (Test-Path -LiteralPath $nativeSdkPath) {
        $nativeSdk=[IO.File]::ReadAllText($nativeSdkPath)
        if ($nativeSdk -match '(?m)^(# )?CONFIG_I2S_ISR_IRAM_SAFE.*$') {
            $nativeSdk=[regex]::Replace($nativeSdk,'(?m)^(# )?CONFIG_I2S_ISR_IRAM_SAFE.*$','CONFIG_I2S_ISR_IRAM_SAFE=y')
        } else { $nativeSdk += "`nCONFIG_I2S_ISR_IRAM_SAFE=y`n" }
        [IO.File]::WriteAllText($nativeSdkPath,$nativeSdk,[Text.UTF8Encoding]::new($false))
    }
}
if ($StayNative) {
    $nativeSdkPath=Join-Path $probeRoot 'sdkconfig'
    if (Test-Path -LiteralPath $nativeSdkPath) {
        $nativeSdk=[IO.File]::ReadAllText($nativeSdkPath)
        $coreChoicePattern='(?m)^(?:# )?(?:CONFIG_BTDM_CTRL_PINNED_TO_CORE_0|CONFIG_BTDM_CTRL_PINNED_TO_CORE_1|CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0|CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1|CONFIG_ESP_MAIN_TASK_AFFINITY_NO_AFFINITY|CONFIG_ESP_TIMER_SHOW_EXPERIMENTAL|CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0|CONFIG_ESP_TIMER_TASK_AFFINITY_CPU1|CONFIG_ESP_TIMER_TASK_AFFINITY_NO_AFFINITY|CONFIG_FREERTOS_TIMER_TASK_AFFINITY_CPU0|CONFIG_FREERTOS_TIMER_TASK_AFFINITY_CPU1|CONFIG_FREERTOS_TIMER_TASK_NO_AFFINITY|CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0|CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1|CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0|CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1|CONFIG_LWIP_TCPIP_TASK_AFFINITY_NO_AFFINITY)(?:=.*| is not set)\r?$'
        $nativeSdk=[regex]::Replace($nativeSdk,$coreChoicePattern,'')
        $nativeSdk += "`nCONFIG_BTDM_CTRL_PINNED_TO_CORE_1=y`nCONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`nCONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y`nCONFIG_FREERTOS_TIMER_TASK_AFFINITY_CPU1=y`nCONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1=y`nCONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`n"
        [IO.File]::WriteAllText($nativeSdkPath,$nativeSdk,[Text.UTF8Encoding]::new($false))
    }
}
# The private probe SDK config follows the explicit observer option. Restore
# the original disabled kernel accounting in non-profile builds.
$statsSdkPath=Join-Path $probeRoot 'sdkconfig'
if (Test-Path -LiteralPath $statsSdkPath) {
    $statsSdk=[IO.File]::ReadAllText($statsSdkPath)
    foreach ($statsKey in @('CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS','CONFIG_FREERTOS_USE_TRACE_FACILITY','CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS','CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID')) {
        $statsPattern='(?m)^(?:# )?'+[regex]::Escape($statsKey)+'(?:=.*| is not set)\r?$'
        $statsSdk=[regex]::Replace($statsSdk,$statsPattern,'')
        $statsSdk += "`n$(if ($TaskRuntimeStats) { $statsKey+'=y' } else { '# '+$statsKey+' is not set' })`n"
    }
    $statsSdk=[regex]::Replace($statsSdk,'(?m)^(?:# )?CONFIG_FREERTOS_(?:RUN_TIME_COUNTER_TYPE_U32|RUN_TIME_COUNTER_TYPE_U64|RUN_TIME_STATS_USING_ESP_TIMER|RUN_TIME_STATS_USING_CPU_CLK)(?:=.*| is not set)\r?$','')
    if ($TaskRuntimeStats) { $statsSdk += "`nCONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32=y`n# CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64 is not set`nCONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER=y`n# CONFIG_FREERTOS_RUN_TIME_STATS_USING_CPU_CLK is not set`n" }
    [IO.File]::WriteAllText($statsSdkPath,$statsSdk,[Text.UTF8Encoding]::new($false))
}
$heapSdkPath=Join-Path $probeRoot 'sdkconfig'
if (Test-Path -LiteralPath $heapSdkPath) {
    $heapSdk=[IO.File]::ReadAllText($heapSdkPath)
    $heapSdk=[regex]::Replace($heapSdk,'(?m)^(?:# )?CONFIG_HEAP_USE_HOOKS(?:=.*| is not set)\r?$','')
    $heapSdk += [Environment]::NewLine + $(if ($HeapAllocTrace) { 'CONFIG_HEAP_USE_HOOKS=y' } else { '# CONFIG_HEAP_USE_HOOKS is not set' }) + [Environment]::NewLine
    [IO.File]::WriteAllText($heapSdkPath,$heapSdk,[Text.UTF8Encoding]::new($false))
}
$env:GIT_NO_LAZY_FETCH = '1'
if (-not (Test-Path -LiteralPath (Join-Path $idfRoot '.git'))) {
    # The bundled SDK is not this host repository's Git checkout.
    $env:GIT_CEILING_DIRECTORIES = $workspaceRoot
}

python (Join-Path $PSScriptRoot 'check_workspace.py')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
. (Join-Path $idfRoot 'export.ps1')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$env:IDF_COMPONENT_MANAGER = '0'
$env:CCACHE_DISABLE = $null
$env:IDF_CCACHE_ENABLE = '1'
$productSecurityBuild=if ($WithProductSecurity) { 'ON' } else { 'OFF' }
$rfcommBuild=if ($WithRfcomm) { 'ON' } else { 'OFF' }
$productIoBuild=if ($WithProductIo) { 'ON' } else { 'OFF' }
$lhdcBuild = if ($WithLhdc) { 'ON' } else { 'OFF' }
$ldacBuild=if ($WithLdac) { 'ON' } else { 'OFF' }
$aacBuild=if ($WithAac) { 'ON' } else { 'OFF' }
$aptxBuild=if ($WithAptx) { 'ON' } else { 'OFF' }
$opusBuild=if ($WithOpus) { 'ON' } else { 'OFF' }
$smpBuild=if ($WithSmp) { 'ON' } else { 'OFF' }
$avrcpBuild=if ($WithAvrcp) { 'ON' } else { 'OFF' }
$sbcBuild = if ($WithSbc) { 'ON' } else { 'OFF' }
$audioBuild = if ($WithAudioProbe) { 'ON' } else { 'OFF' }
$outputBuild = if ($WithAudioOutput) { 'ON' } else { 'OFF' }
$gattBuild = if ($WithGatt) { 'ON' } else { 'OFF' }
$securityBuild = if ($WithClassicSecurity) { 'ON' } else { 'OFF' }
$ingressBuild = if ($TraceIngress) { 'ON' } else { 'OFF' }
$disableReceiveFlow = if ($WithoutRxFlow) { 'ON' } else { 'OFF' }
$keepOwnedBuild = if ($KeepOwned) { 'ON' } else { 'OFF' }
$firCompilerBuild = if ($FirCompilerCheck) { 'ON' } else { 'OFF' }
$dspStageBuild = if ($DspStageProfile) { 'ON' } else { 'OFF' }
$audioStageBuild = if ($AudioStageProfile) { 'ON' } else { 'OFF' }
$pipelineBuild = if ($PipelineDiagnostic) { 'ON' } else { 'OFF' }
$codecDiagnosticBuild = if ($CodecDiagnosticOnly) { 'ON' } else { 'OFF' }
$deferredCodecBuild = if ($DeferredCodecCheck) { 'ON' } else { 'OFF' }
$asyncAudioBuild = if ($AsyncAudio) { 'ON' } else { 'OFF' }
$rfDiagnosticBuild = if ($RfProtocolDiagnostic) { 'ON' } else { 'OFF' }
$lhdcMaxBuild = if ($LhdcMaximumCheck) { 'ON' } else { 'OFF' }
$lhdcStageBuild = if ($LhdcStageProfile) { 'ON' } else { 'OFF' }
$peerBytes = '0,0,0,0,0,0'
if ($WithClassicSecurity -and $TestPeer) {
    $peerOctets=$TestPeer.Split(':')
    [array]::Reverse($peerOctets)
    $peerBytes=($peerOctets | ForEach-Object { '0x'+$_ }) -join ','
}
$idfArguments=@('--ccache','-C',$probeRoot,'-B',$buildRoot,"-DDBT_PROBE_PRODUCT_SECURITY=$productSecurityBuild", "-DDBT_PROBE_RFCOMM=$rfcommBuild", "-DDBT_PROBE_PRODUCT_IO=$productIoBuild", "-DDBT_PROBE_LHDC_BUILD=$lhdcBuild", "-DDBT_PROBE_OPUS_BUILD=$opusBuild", "-DDBT_PROBE_AAC_BUILD=$aacBuild", "-DDBT_PROBE_LDAC_BUILD=$ldacBuild", "-DDBT_PROBE_APTX_BUILD=$aptxBuild", "-DDBT_PROBE_SMP=$smpBuild", "-DDBT_PROBE_AVRCP=$avrcpBuild", "-DDBT_PROBE_SBC_BUILD=$sbcBuild", "-DDBT_PROBE_AUDIO_TEST=$audioBuild", "-DDBT_PROBE_AUDIO_OUTPUT=$outputBuild", "-DDBT_PROBE_KEEP_OWNED_ACTIVE=$keepOwnedBuild", "-DDBT_PROBE_GATT_TEST=$gattBuild", "-DDBT_PROBE_CLASSIC_SECURITY_TEST=$securityBuild", "-DDBT_PROBE_INGRESS_TRACE=$ingressBuild", "-DDBT_PROBE_DISABLE_RX_FLOW=$disableReceiveFlow", "-DDBT_PROBE_PAIRING_PEER_BYTES=$peerBytes")
if ($LhdcSourceDir) { $idfArguments += "-DLHDCV5_SOURCE_DIR=$LhdcSourceDir" }
$idfArguments += "-DDBT_PROBE_FIR_COMPILER_CHECK=$firCompilerBuild"
$idfArguments += "-DDBT_PROBE_DSP_STAGE_PROFILE=$dspStageBuild"
$idfArguments += "-DDBT_PROBE_DSP_INLINE_CHECK=$(if ($DspInlineCheck) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_AUDIO_STAGE_PROFILE=$audioStageBuild"
$idfArguments += "-DDBT_PROBE_PIPELINE_DIAGNOSTIC=$pipelineBuild"
$idfArguments += "-DDBT_PROBE_CODEC_DIAGNOSTIC_ONLY=$codecDiagnosticBuild"
$idfArguments += "-DDBT_PROBE_DEFERRED_CODEC_CHECK=$deferredCodecBuild"
$idfArguments += "-DDBT_PROBE_ASYNC_AUDIO=$asyncAudioBuild"
$idfArguments += "-DDBT_PROBE_PCM_LOANS=$(if ($PcmLoans) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_APTX_SNAPSHOT=$(if ($AptxSnapshot) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_PCM_COPIED=$(if ($PcmCopied) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_PCM_TARGET_MS=$PcmTargetMs"
$idfArguments += "-DDBT_PROBE_PCM_HASH=$(if ($PcmHash) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_BLOCK_DSP=$(if ($BlockDsp) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_WORD_DSP=$(if ($WordDsp) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_STAY_NATIVE=$(if ($StayNative) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_STORAGE_RESERVE=$(if ($StorageReserve) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_RF_PROTOCOL_DIAGNOSTIC=$rfDiagnosticBuild"
$idfArguments += "-DDBT_PROBE_DECODER_CORE=$DecoderCore"
$idfArguments += "-DDBT_PROBE_DECODER_PRIORITY=$DecoderPriority"
$idfArguments += "-DDBT_PROBE_PRIORITY_ORDER=$(if ($PriorityOrder) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_LOGGER_CORE=$LoggerCore"
$idfArguments += "-DDBT_PROBE_CONSOLE_IRQ=$(if ($ConsoleIrq) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_HEAP_ALLOC_TRACE=$(if ($HeapAllocTrace) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_TASK_RUNTIME_STATS=$(if ($TaskRuntimeStats) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_TASK_RUNTIME_ONLY=$(if ($TaskRuntimeOnly) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_TASK_CYCLE_ACCOUNTING=$(if ($TaskCycleAccounting) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_DISPATCH_TRACE=$(if ($DispatchTrace) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_SCHEDULER_TRACE=$(if ($SchedulerTrace) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_CODEC_AWARE_RENDER=$(if ($CodecAwareRender) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_HIGH_RATE_PCM_REBALANCE=$(if ($HighRatePcmRebalance) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_EARLY_INPUT_RELEASE=$(if ($EarlyInputRelease) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_MATCHED_FEEDBACK=$(if ($MatchedFeedback) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_MATCHED_FEEDBACK_COMPARE=$(if ($MatchedFeedbackCompare) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_RENDER_FAIR_SHARE=$(if ($RenderFairShare) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_VOLUME_RAMP_BLOCK=$(if ($VolumeRampBlock) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_VOLUME_RAMP_COMPARE=$(if ($VolumeRampCompare) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_OWNER_PACER=$(if ($OwnerPacer) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_DSP_ASSIST=$(if ($DspAssist) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_DSP_ASSIST_BLOCKS=$DspAssistBlocks"
$idfArguments += "-DDBT_PROBE_DSP_BLOCK_IRAM=$(if ($DspBlockIram) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_UNITY_PCM_COPY=$(if ($UnityPcmCopy) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_LHDC_CORE0_RENDER=$(if ($LhdcCore0Render) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_LHDC_FAST_MANTISSA=$(if ($LhdcFastMantissa) { 'ON' } else { 'OFF' })"
$idfArguments += "-DDBT_PROBE_LHDC_MAX_CHECK=$lhdcMaxBuild"
$idfArguments += "-DDBT_PROBE_LHDC_STAGE_PROFILE=$lhdcStageBuild"
$sourceManifestPath=Join-Path $buildRoot 'local-build-sources.json'
# Reconfigure first so generated ports, compilation flags and SDK configuration
# are final before fingerprinting the actual build inputs.
python (Join-Path $idfRoot 'tools\idf.py') @idfArguments reconfigure
$buildExitCode=$LASTEXITCODE
if ($buildExitCode -eq 0) {
    $generatedConsoleConfig=[IO.File]::ReadAllText((Join-Path $buildRoot 'config/sdkconfig.h'))
    $generatedConsoleMatch=[regex]::Match($generatedConsoleConfig,'(?m)^#define CONFIG_ESP_CONSOLE_UART_BAUDRATE (\d+)\r?$')
    if (-not $generatedConsoleMatch.Success -or [int]$generatedConsoleMatch.Groups[1].Value -ne $ConsoleBaud) { throw 'Requested console baud does not match generated SDK configuration' }
}
$sourceCheckCode=-1
$invalidationExitCode=-1
if ($buildExitCode -eq 0) {
    # Restored files can be older than objects. Re-enter every compiler through
    # ccache; a stable source manifest alone does not prove Ninja recompiled it.
    python (Join-Path $PSScriptRoot 'invalidate_compile_outputs.py') --build $buildRoot --receipt (Join-Path $buildRoot 'local-build-invalidation.json')
    $invalidationExitCode=$LASTEXITCODE
    $buildExitCode=$invalidationExitCode
}
if ($buildExitCode -eq 0) {
    python (Join-Path $PSScriptRoot 'source_snapshot.py') --save $sourceManifestPath
    $buildExitCode=$LASTEXITCODE
    if ($buildExitCode -eq 0) {
        python (Join-Path $idfRoot 'tools\idf.py') --ccache -C $probeRoot -B $buildRoot build
        $buildExitCode=$LASTEXITCODE
        python (Join-Path $PSScriptRoot 'source_snapshot.py') --verify $sourceManifestPath
        $sourceCheckCode=$LASTEXITCODE
    }
}
python (Join-Path $PSScriptRoot 'check_workspace.py')
$workspaceCheckCode=$LASTEXITCODE
$buildReceipt=[ordered]@{schema=2;console_baud=$ConsoleBaud;compiled_inputs_rechecked=($invalidationExitCode -eq 0);invalidation_exit=$invalidationExitCode;passed=($buildExitCode -eq 0 -and $sourceCheckCode -eq 0 -and $workspaceCheckCode -eq 0 -and $invalidationExitCode -eq 0);state='failed';hardware_accessed=$false;product_io=[bool]$WithProductIo;product_security=[bool]$WithProductSecurity;rfcomm=[bool]$WithRfcomm;build_exit=$buildExitCode;source_check_exit=$sourceCheckCode;workspace_check_exit=$workspaceCheckCode}
$buildReceipt.fir_compiler_check=[bool]$FirCompilerCheck
$buildReceipt.dsp_stage_profile=[bool]$DspStageProfile
$buildReceipt.dsp_inline_check=[bool]$DspInlineCheck
$buildReceipt.audio_stage_profile=[bool]$AudioStageProfile
$buildReceipt.aptx_snapshot=[bool]$AptxSnapshot
$buildReceipt.pipeline_diagnostic=[bool]$PipelineDiagnostic
$buildReceipt.codec_diagnostic_only=[bool]$CodecDiagnosticOnly
$buildReceipt.deferred_codec_check=[bool]$DeferredCodecCheck
$buildReceipt.async_audio=[bool]$AsyncAudio
$buildReceipt.pcm_loans=[bool]$PcmLoans
$buildReceipt.pcm_copied=[bool]$PcmCopied
$buildReceipt.pcm_hash=[bool]($PcmHash -or -not $StayNative)
$buildReceipt.pcm_target_ms=$PcmTargetMs
$buildReceipt.block_dsp=[bool]$BlockDsp
$buildReceipt.word_dsp=[bool]$WordDsp
$buildReceipt.stay_native=[bool]$StayNative
$buildReceipt.storage_reserve=[bool]$StorageReserve
$buildReceipt.rf_protocol_diagnostic=[bool]$RfProtocolDiagnostic
$buildReceipt.decoder_core=$DecoderCore
$buildReceipt.bluetooth_core=1
$buildReceipt.renderer_core=1
$buildReceipt.background_core=1
$buildReceipt.decoder_priority=$DecoderPriority
$buildReceipt.priority_order=[bool]$PriorityOrder
$buildReceipt.host_priority=$(if ($StayNative) { 22 } elseif ($PriorityOrder) { 7 } else { 6 })
$buildReceipt.renderer_priority=$(if ($RenderFairShare) { 22 } elseif ($StayNative) { 20 } elseif ($PriorityOrder) { 5 } else { 6 })
$buildReceipt.auxiliary_renderer_priority=$(if ($StayNative) { 20 } elseif ($PriorityOrder) { 5 } else { 6 })
$buildReceipt.owner_ready_yield=($RenderFairShare -or $DecoderPriority -eq $buildReceipt.host_priority)
$buildReceipt.logger_core=$LoggerCore
$buildReceipt.console_irq=[bool]$ConsoleIrq
$buildReceipt.heap_alloc_trace=[bool]$HeapAllocTrace
$buildReceipt.task_runtime_stats=[bool]$TaskRuntimeStats
$buildReceipt.task_runtime_only=[bool]$TaskRuntimeOnly
$buildReceipt.task_cycle_accounting=[bool]$TaskCycleAccounting
$buildReceipt.dispatch_trace=[bool]$DispatchTrace
$buildReceipt.scheduler_trace=[bool]$SchedulerTrace
$buildReceipt.codec_aware_render=[bool]$CodecAwareRender
$buildReceipt.high_rate_pcm_rebalance=[bool]$HighRatePcmRebalance
$buildReceipt.early_input_release=[bool]$EarlyInputRelease
$buildReceipt.matched_feedback=[bool]$MatchedFeedback
$buildReceipt.matched_feedback_compare=[bool]$MatchedFeedbackCompare
$buildReceipt.render_fair_share=[bool]$RenderFairShare
$buildReceipt.volume_ramp_block=[bool]$VolumeRampBlock
$buildReceipt.volume_ramp_compare=[bool]$VolumeRampCompare
$buildReceipt.owner_pacer=[bool]$OwnerPacer
$buildReceipt.dsp_assist=[bool]$DspAssist
$buildReceipt.dsp_assist_blocks=[int]$DspAssistBlocks
$buildReceipt.dsp_block_iram=[bool]$DspBlockIram
$buildReceipt.unity_pcm_copy=[bool]$UnityPcmCopy
$buildReceipt.lhdc_core0_render=[bool]$LhdcCore0Render
$buildReceipt.lhdc_fast_mantissa=[bool]$LhdcFastMantissa
$buildReceipt.lhdc_maximum_check=[bool]$LhdcMaximumCheck
$buildReceipt.lhdc_stage_profile=[bool]$LhdcStageProfile
if ($buildReceipt.passed) {
    $buildReceipt.state='complete'
    $buildReceipt.source_manifest_sha256=(Get-FileHash -LiteralPath $sourceManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.image_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'native_bt_probe.bin') -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.config_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'config/sdkconfig.h') -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.partition_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'partition_table/partition-table.bin') -Algorithm SHA256).Hash.ToLowerInvariant()
}
$buildReceipt | ConvertTo-Json | Set-Content -LiteralPath $buildReceiptPath -Encoding utf8
if ($buildExitCode -ne 0) { exit $buildExitCode }
if ($sourceCheckCode -ne 0) { exit $sourceCheckCode }
Write-Host 'OFF-BOARD BUILD: intended destination is existing ota_1 at 0x390000 (0x280000 bytes).'
Write-Host 'Generic SDK flash hints are disabled for this project. Preserve bootloader/table/recovery; use the admission helper only after the user releases HARDWARE_HOLD.'
exit $workspaceCheckCode
