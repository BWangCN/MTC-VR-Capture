# Copy the staged build to the established NatNet/firewall path.
# This script never starts, stops, or restarts motive_client.
$ErrorActionPreference = 'Stop'
$taskProject = $PSScriptRoot
$taskRelease = Join-Path $taskProject 'build/Release'
$taskTarget = [IO.Path]::GetFullPath((Join-Path $taskRelease 'motive_client.exe'))
$taskStaged = Join-Path $taskRelease 'motive_client.recording-update.exe'
if (-not (Test-Path -LiteralPath $taskStaged)) { throw "Staged executable not found: $taskStaged" }
$taskRunning = Get-CimInstance Win32_Process -Filter "Name = 'motive_client.exe'" |
    Where-Object { $_.ExecutablePath -and [IO.Path]::GetFullPath($_.ExecutablePath) -eq $taskTarget }
if ($taskRunning) {
    throw 'motive_client is running. Stop it yourself, then run this script again. No processes were changed.'
}
if (Test-Path -LiteralPath $taskTarget) {
    $taskBackup = Join-Path $taskRelease ('motive_client.before-recording-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.exe')
    Copy-Item -LiteralPath $taskTarget -Destination $taskBackup
}
Copy-Item -LiteralPath $taskStaged -Destination $taskTarget -Force
Write-Host "Installed: $taskTarget"
Write-Host 'Start motive_client yourself with your usual local IP argument. No process was started.'
