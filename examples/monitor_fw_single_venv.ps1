param(
    [int]$PollSeconds = 20,
    [switch]$KillExisting = $true,
    [int]$RestartDelaySeconds = 5
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$scriptName = 'run_rated_speed_field_weakening_search.py'
$venvPython = Join-Path $root '.venv\Scripts\python.exe'
$stdoutLog = Join-Path $root 'data\logs\fw_live.log'
$stderrLog = Join-Path $root 'data\logs\fw_live.err.log'
$venvPythonNorm = $venvPython.ToLower()

if (-not (Test-Path $venvPython)) {
    throw "Venv Python not found: $venvPython"
}

function Stop-SearchInstances {
    $running = Get-CimInstance Win32_Process |
        Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -like "*$scriptName*" }
    foreach ($p in $running) {
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
    return @($running).Count
}

if ($KillExisting) {
    $stopped = Stop-SearchInstances
    Write-Host "Stopped existing instances: $stopped"
}

$attempt = 0
while ($true) {
    $attempt += 1

    if (Test-Path $stdoutLog) { Remove-Item $stdoutLog -Force }
    if (Test-Path $stderrLog) { Remove-Item $stderrLog -Force }

    $stoppedBeforeStart = Stop-SearchInstances
    if ($stoppedBeforeStart -gt 0) {
        Write-Host "Cleaned up stale instances before start: $stoppedBeforeStart"
    }

    $proc = Start-Process -FilePath $venvPython `
        -ArgumentList '-u', (Join-Path 'examples' $scriptName) `
        -WorkingDirectory $root `
        -RedirectStandardOutput $stdoutLog `
        -RedirectStandardError $stderrLog `
        -PassThru

    Write-Host ("Attempt {0}: started PID={1} using venv python: {2}" -f $attempt, $proc.Id, $venvPython)
    Write-Host ("Stdout log: {0}" -f $stdoutLog)
    Write-Host ("Stderr log: {0}" -f $stderrLog)

    Start-Sleep -Seconds 1
    $nonVenvDupes = Get-CimInstance Win32_Process |
        Where-Object {
            $_.Name -eq 'python.exe' -and
            $_.CommandLine -like "*$scriptName*" -and
            $_.CommandLine -and
            ($_.CommandLine.ToLower() -notmatch [regex]::Escape($venvPythonNorm))
        }
    foreach ($d in $nonVenvDupes) {
        Stop-Process -Id $d.ProcessId -Force -ErrorAction SilentlyContinue
    }
    if (@($nonVenvDupes).Count -gt 0) {
        Write-Host "Killed non-venv duplicates: $(@($nonVenvDupes).Count)"
    }

    $lastLine = ''
    while (-not $proc.HasExited) {
        Start-Sleep -Seconds $PollSeconds

        # Enforce singleton: kill any non-venv duplicate that appears later.
        $dupesDuringRun = Get-CimInstance Win32_Process |
            Where-Object {
                $_.Name -eq 'python.exe' -and
                $_.CommandLine -like "*$scriptName*" -and
                $_.CommandLine -and
                ($_.CommandLine.ToLower() -notmatch [regex]::Escape($venvPythonNorm))
            }
        foreach ($d in $dupesDuringRun) {
            Stop-Process -Id $d.ProcessId -Force -ErrorAction SilentlyContinue
        }
        if (@($dupesDuringRun).Count -gt 0) {
            Write-Host "Killed non-venv duplicates during run: $(@($dupesDuringRun).Count)"
        }

        if (-not (Test-Path $stdoutLog)) {
            continue
        }

        $newLine = Get-Content $stdoutLog -Tail 120 |
            Where-Object {
                $_ -match 'PHASE_PROGRESS|TUNING_PROGRESS|CONVERGENCE_FOUND|NO_CONVERGENCE_FOUND|PHASE_SELECTION_DONE'
            } |
            Select-Object -Last 1

        if ($newLine -and $newLine -ne $lastLine) {
            $lastLine = $newLine
            Write-Host ((Get-Date).ToString('HH:mm:ss') + ' | ' + $newLine)
        }
    }

    Write-Host ("Attempt {0}: process exited with code {1}" -f $attempt, $proc.ExitCode)

    $hasTerminalState = $false
    if (Test-Path $stdoutLog) {
        $hasTerminalState = Select-String -Path $stdoutLog -Pattern 'CONVERGENCE_FOUND|NO_CONVERGENCE_FOUND' -Quiet
    }

    if ($hasTerminalState) {
        Write-Host 'Terminal state reached in stdout log; stopping supervisor.'
        Write-Host '--- Final stdout tail ---'
        Get-Content $stdoutLog -Tail 30
        if (Test-Path $stderrLog) {
            Write-Host '--- Final stderr tail ---'
            Get-Content $stderrLog -Tail 20
        }
        break
    }

    Write-Host ("Attempt {0}: no terminal state detected, restarting in {1}s..." -f $attempt, $RestartDelaySeconds)
    Start-Sleep -Seconds $RestartDelaySeconds
}
