# Run on the Windows laptop after landing and the mission process has exited.
[CmdletBinding()]
param(
    [string]$Destination = 'F:\工创比赛\录像',
    [switch]$SyncOnly,
    [switch]$NoPlay
)
$ErrorActionPreference = 'Stop'
Get-Command ssh, scp -ErrorAction Stop | Out-Null
$computerUnix = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
$sync = 'python3 "$HOME/simple_flight_ws/src/simple_flight_payload/scripts/recording_name.py" --sync-unix ' + $computerUnix
if ($SyncOnly) {
    & ssh jetson@10.42.0.1 $sync
    if ($LASTEXITCODE -ne 0) { throw 'Recording filename clock sync failed.' }
    return
}
# A marker keeps SSH banners and shutdown messages separate from the filename.
$remote = @'
set -e
bash "$HOME/simple_flight_ws/competitionctl.sh" stop-vision
__SYNC__
python3 - <<'PY'
from pathlib import Path
import re
root=Path.home()/'simple_flight_ws/log'
pointer=root/'latest_recording.txt'
name=pointer.read_text().strip() if pointer.exists() else ''
if not re.fullmatch(r'competition_[A-Za-z0-9_-]+\.avi', name) or not (root/name).is_file():
    files=list(root.glob('competition_*.avi'))
    if not files: raise SystemExit('No recording found.')
    name=max(files,key=lambda p:p.stat().st_mtime_ns).name
print('LATEST_VIDEO='+name)
PY
'@
$remote = $remote.Replace('__SYNC__', $sync).Replace("`r`n", "`n")
$output = @(& ssh jetson@10.42.0.1 $remote)
if ($LASTEXITCODE -ne 0) { throw ('Recording export preparation failed: ' + ($output -join "`n")) }
$marker = @($output | Where-Object { $_ -match '^LATEST_VIDEO=competition_[A-Za-z0-9_-]+\.avi$' })
if ($marker.Count -ne 1) { throw 'No unique finalized recording filename was returned.' }
$name = $marker[0].Substring('LATEST_VIDEO='.Length)
New-Item -ItemType Directory -Force -Path $Destination | Out-Null
$Destination = (Resolve-Path -LiteralPath $Destination).Path
$local = Join-Path $Destination $name
& scp ("jetson@10.42.0.1:/home/jetson/simple_flight_ws/log/" + $name) $local
if ($LASTEXITCODE -ne 0) { throw 'Video transfer failed; the Nano copy has been kept.' }
Write-Host ('Saved: ' + $local)
Write-Host 'Preview is stopped to close the AVI. The next competitionctl.sh run starts it again.'
if (-not $NoPlay) { Invoke-Item -LiteralPath $local }
