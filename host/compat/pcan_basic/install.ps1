param(
    [string]$InstallDir = "${env:ProgramFiles(x86)}\EYouPlanet(RP Series)",
    [switch]$Restore
)
$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $arguments = '-NoProfile -ExecutionPolicy Bypass -NoExit -File "' + $PSCommandPath +
        '" -InstallDir "' + $InstallDir + '"'
    if ($Restore) { $arguments += ' -Restore' }
    Start-Process powershell.exe -Verb RunAs -ArgumentList $arguments
    return
}
if (Get-Process -Name 'eyou_canfdx' -ErrorAction SilentlyContinue) {
    throw 'Close EYou RP Series before replacing its DLL.'
}
$target = Join-Path $InstallDir 'PCANBasic.dll'
$backup = Join-Path $InstallDir 'PCANBasic.before-libhcs.dll'
if (-not (Test-Path (Join-Path $InstallDir 'eyou_canfdx.exe'))) {
    throw 'EYou installation not found. Specify -InstallDir with its folder.'
}
if ($Restore) {
    if (-not (Test-Path $backup)) { throw 'Original DLL backup was not found.' }
    Copy-Item -LiteralPath $backup -Destination $target -Force
    Write-Host 'Original DLL restored. Use the matching original firmware.'
} else {
    $source = Join-Path $PSScriptRoot 'PCANBasic.dll'
    if (-not (Test-Path $source)) { throw 'PCANBasic.dll must be next to this script.' }
    if (Test-Path $target) {
        if ((Get-FileHash $source).Hash -eq (Get-FileHash $target).Hash) {
            Write-Host 'HCS compatibility DLL is already installed.'
            return
        }
        if (-not (Test-Path $backup)) {
            Copy-Item -LiteralPath $target -Destination $backup
        }
    }
    Copy-Item -LiteralPath $source -Destination $target -Force
    Write-Host 'HCS CAN FD compatibility DLL installed. CAN1 = PCAN_USBBUS1.'
    Write-Host 'Requires matching HCS WinUSB firmware. No PEAK driver is used.'
}
