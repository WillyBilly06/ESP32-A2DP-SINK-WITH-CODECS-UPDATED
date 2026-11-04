param(
    [string]$AndroidSdk = "$env:LOCALAPPDATA\Android\Sdk",
    [string]$BuildToolsVersion = '36.0.0'
)
$ErrorActionPreference = 'Stop'
$checkRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$checkBuild = Join-Path $checkRoot 'build\android-feature-check'
$androidJar = Join-Path $AndroidSdk 'platforms\android-36\android.jar'
$androidTools = Join-Path $AndroidSdk "build-tools\$BuildToolsVersion"
$checkClasses = Join-Path $checkBuild 'classes'
$checkDex = Join-Path $checkBuild 'dex'
New-Item -ItemType Directory -Force -Path $checkClasses,$checkDex | Out-Null
& javac --release 8 -cp $androidJar -d $checkClasses (Join-Path $PSScriptRoot 'FeatureCheck.java')
if ($LASTEXITCODE) { throw 'Java compilation failed' }
$classFiles = @(Get-ChildItem -LiteralPath $checkClasses -Recurse -Filter '*.class' | ForEach-Object FullName)
& "$androidTools\d8.bat" --lib $androidJar --output $checkDex @classFiles
if ($LASTEXITCODE) { throw 'DEX compilation failed' }
$rawApk = Join-Path $checkBuild 'raw.apk'
$alignedApk = Join-Path $checkBuild 'aligned.apk'
$checkApk = Join-Path $checkBuild 'bdk-firmware-check.apk'
& "$androidTools\aapt2.exe" link -o $rawApk --manifest (Join-Path $PSScriptRoot 'AndroidManifest.xml') -I $androidJar
if ($LASTEXITCODE) { throw 'APK packaging failed' }
& jar uf $rawApk -C $checkDex classes.dex
if ($LASTEXITCODE) { throw 'DEX packaging failed' }
& "$androidTools\zipalign.exe" -f 4 $rawApk $alignedApk
if ($LASTEXITCODE) { throw 'APK alignment failed' }
$checkKey = Join-Path $checkBuild 'test.keystore'
if (-not (Test-Path -LiteralPath $checkKey)) {
    & keytool -genkeypair -keystore $checkKey -storepass android -keypass android -alias test -keyalg RSA -keysize 2048 -validity 30 -dname 'CN=Local Firmware Test'
    if ($LASTEXITCODE) { throw 'Local test key creation failed' }
}
& "$androidTools\apksigner.bat" sign --ks $checkKey --ks-pass pass:android --key-pass pass:android --out $checkApk $alignedApk
if ($LASTEXITCODE) { throw 'APK signing failed' }
Write-Output $checkApk
