param([string]$Port, [switch]$List, [switch]$DryRun, [switch]$Yes)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
try {
    function Get-Candidates {
        @(Get-CimInstance Win32_SerialPort | Where-Object {
            $_.PNPDeviceID -match '^USB\\VID_303A&PID_(1001|821E)'
        })
    }
    $candidates = @(Get-Candidates)
    $candidates | ForEach-Object { Write-Host "$($_.DeviceID) $($_.PNPDeviceID)" }
    if ($List) { exit 0 }
    if ($Port) { $selected = @($candidates | Where-Object { $_.DeviceID -eq $Port }) }
    else { $selected = $candidates }
    if ($selected.Count -ne 1) { throw 'Select exactly one USB candidate with -Port COM8. Model is not verified by USB ID.' }
    $device = $selected[0]
    $files = @('tool/esptool.exe', 'firmware/app.bin', 'firmware/bootloader.bin', 'firmware/partitions.bin', 'firmware/boot_app0.bin')
    $checksums = @{}
    Get-Content -LiteralPath SHA256SUMS | ForEach-Object {
        $parts = $_ -split '\s+', 2
        if ($parts.Count -eq 2) { $checksums[$parts[1]] = $parts[0] }
    }
    foreach ($file in $files) {
        $actual = (Get-FileHash -Algorithm SHA256 -LiteralPath $file).Hash
        if (!$checksums.ContainsKey($file) -or $actual -ne $checksums[$file]) { throw "Checksum failed: $file" }
    }
    Write-Host "Firmware: $(Get-Content VERSION). Confirm this is Waveshare S3 Touch LCD 1.69."
    if ($DryRun) { Write-Host 'DRY RUN: checksums OK; no reset or write.'; exit 0 }
    if (!$Yes -and (Read-Host "Type UPDATE to flash $($device.DeviceID)") -cne 'UPDATE') { exit 1 }
    $current = @(Get-Candidates | Where-Object { $_.PNPDeviceID -eq $device.PNPDeviceID })
    if ($current.Count -ne 1) { throw 'Original USB device disappeared or is ambiguous; rediscover.' }
    & '.\tool\esptool.exe' --chip esp32s3 --port $current[0].DeviceID --baud 460800 --before default-reset --after hard-reset `
        write-flash --flash-mode keep --flash-freq keep --flash-size keep `
        0x0 firmware/bootloader.bin 0x8000 firmware/partitions.bin 0xe000 firmware/boot_app0.bin 0x10000 firmware/app.bin
    if ($LASTEXITCODE -ne 0) { throw "esptool failed with exit code $LASTEXITCODE" }
    Write-Host 'Upload complete; reset requested. Check version, touch and CAL on screen.'
    exit 0
} catch {
    Write-Error -ErrorAction Continue $_
    Write-Host 'Close serial monitors. If needed hold BOOT, tap RESET, release BOOT and rediscover.'
    exit 1
}
