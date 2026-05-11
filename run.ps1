# Startup script for traffic_sim (Bevy 0.18.1 / GNU toolchain)
# Adds MinGW64 dlltool to PATH, then builds and runs the simulation.

$mingwBin = "C:\msys64\mingw64\bin"
if (-not ($env:PATH -split ";" | Where-Object { $_ -eq $mingwBin })) {
    $env:PATH = "$mingwBin;" + $env:PATH
}

$projectDir = $PSScriptRoot
Push-Location $projectDir

$configPath = Join-Path $projectDir "sim_config.toml"

function Get-TomlScalar {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text,
        [Parameter(Mandatory = $true)]
        [string]$Key
    )

    foreach ($line in ($Text -split "`n")) {
        $trimmed = $line.Trim()
        if ($trimmed -eq "" -or $trimmed.StartsWith("#") -or $trimmed.StartsWith("[")) {
            continue
        }

        if ($trimmed -match ("^" + [regex]::Escape($Key) + "\s*=\s*(.+)$")) {
            return $Matches[1].Trim()
        }
    }

    return $null
}

if (Test-Path $configPath) {
    $configText = Get-Content $configPath -Raw
    $fixedDt = Get-TomlScalar -Text $configText -Key "fixed_dt"
    $maxFrames = Get-TomlScalar -Text $configText -Key "max_frames"
    $approachRadius = Get-TomlScalar -Text $configText -Key "approach_radius"
    $clearRadius = Get-TomlScalar -Text $configText -Key "clear_radius"
    $maxSpeed = Get-TomlScalar -Text $configText -Key "max_speed"

    Write-Host "==> Key config from sim_config.toml" -ForegroundColor Yellow
    Write-Host "    fixed_dt=$fixedDt, max_frames=$maxFrames, approach_radius=$approachRadius, clear_radius=$clearRadius, max_speed=$maxSpeed"
} else {
    Write-Host "==> sim_config.toml not found; app will create defaults on first run." -ForegroundColor Yellow
}

Write-Host "==> Building traffic_sim..." -ForegroundColor Cyan
cargo build
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed." -ForegroundColor Red
    Pop-Location
    exit $LASTEXITCODE
}

Write-Host "==> Running traffic_sim..." -ForegroundColor Green
cargo run

Pop-Location
