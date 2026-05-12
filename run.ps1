# Startup script for traffic_sim (Tauri 2 + Pixi.js)
# Adds MinGW64 to PATH, prints config summary, then launches the app.

$mingwBin = "C:\msys64\mingw64\bin"
if (-not ($env:PATH -split ";" | Where-Object { $_ -eq $mingwBin })) {
    $env:PATH = "$mingwBin;" + $env:PATH
}

$llvmBin = "C:\Program Files\LLVM\bin"
if (-not ($env:PATH -split ";" | Where-Object { $_ -eq $llvmBin })) {
    $env:PATH = "$llvmBin;" + $env:PATH
}

$projectDir = $PSScriptRoot
Push-Location $projectDir

$configPath = Join-Path $projectDir "src-tauri\sim_config.toml"

function Get-TomlScalar {
    param([string]$Text, [string]$Key)
    foreach ($line in ($Text -split "`n")) {
        $t = $line.Trim()
        if ($t -eq "" -or $t.StartsWith("#") -or $t.StartsWith("[")) { continue }
        if ($t -match ("^" + [regex]::Escape($Key) + "\s*=\s*([^#]+)")) {
            return $Matches[1].Trim()
        }
    }
    return "(not found)"
}

if (Test-Path $configPath) {
    $cfg = Get-Content $configPath -Raw
    Write-Host ""
    Write-Host "==> sim_config.toml" -ForegroundColor Yellow
    Write-Host ("    approach_radius=" + (Get-TomlScalar $cfg "approach_radius") +
                "  clear_radius=" + (Get-TomlScalar $cfg "clear_radius"))
    Write-Host ("    a_max=" + (Get-TomlScalar $cfg "a_max") +
                "  b_comf=" + (Get-TomlScalar $cfg "b_comf") +
                "  s0=" + (Get-TomlScalar $cfg "s0") +
                "  t_headway=" + (Get-TomlScalar $cfg "t_headway"))
    Write-Host ("    max_speed=" + (Get-TomlScalar $cfg "max_speed") +
                "  stop_line_offset=" + (Get-TomlScalar $cfg "stop_line_offset"))
    Write-Host ""
} else {
    Write-Host "==> sim_config.toml not found - defaults will be created on first run." -ForegroundColor Yellow
}

if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: npm not found. Install Node.js from https://nodejs.org" -ForegroundColor Red
    Pop-Location; exit 1
}

if (-not (Test-Path "node_modules")) {
    Write-Host "==> Installing npm dependencies..." -ForegroundColor Cyan
    npm install
    if ($LASTEXITCODE -ne 0) { Pop-Location; exit $LASTEXITCODE }
}

Write-Host "==> Starting Tauri dev mode (first run compiles Rust - may take a minute)..." -ForegroundColor Cyan
npm run dev

Pop-Location
