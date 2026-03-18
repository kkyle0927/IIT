param(
    [switch]$RunAll,
    [switch]$CompareResults,
    [string[]]$Configs
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

$defaultOrder = @(
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_core.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_core_dropout005.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_core_standing_weight.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_deflection_asymmetry.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_gaitphase_asymmetry.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_mechpower_kinematics.yaml",
    "configs/archive_season2/archive_1/exp_S2A1_bodyframe_biomech_fusion.yaml"
)

$pythonExe = "C:\Users\ChanyoungKo\anaconda3\envs\IIT\python.exe"
if (-not (Test-Path $pythonExe)) {
    throw "IIT python not found: $pythonExe"
}

if (-not $Configs -or $Configs.Count -eq 0) {
    if ($RunAll) {
        $Configs = $defaultOrder
    } else {
        $Configs = @($defaultOrder[0])
    }
}

$logDir = Join-Path $PSScriptRoot "logs/local_season2_archive1"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$env:PYTHONUNBUFFERED = "1"

foreach ($config in $Configs) {
    $configPath = Join-Path $PSScriptRoot $config
    if (-not (Test-Path $configPath)) {
        throw "Config not found: $config"
    }

    $configName = [System.IO.Path]::GetFileNameWithoutExtension($configPath)
    $logPath = Join-Path $logDir "$configName.log"

    Write-Host ""
    Write-Host "=== Running $configName ==="
    Write-Host "Config: $config"
    Write-Host "Log   : $logPath"

    & $pythonExe model_training.py --config $configPath 2>&1 | Tee-Object -FilePath $logPath

    if ($LASTEXITCODE -ne 0) {
        throw "Training failed for $config"
    }
}

if ($CompareResults) {
    Write-Host ""
    Write-Host "=== Running compare_results.py --auto ==="
    & $pythonExe compare_results.py --auto
    if ($LASTEXITCODE -ne 0) {
        throw "compare_results.py --auto failed"
    }
}
