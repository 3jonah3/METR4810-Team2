param([string]$LccRoot = 'C:/Program Files/MATLAB/R2026a/sys/lcc64/lcc64',
      [ValidateSet(0,1500)][int]$OvertravelSteps = 0)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot/..").Path
$output = Join-Path $root 'METR4810_Team2_code/build/host'
New-Item -ItemType Directory -Force $output | Out-Null
$inputs = @(Join-Path $root 'tests/host/test_control.c') +
    @('robot_control','serial','motor','stepper','servo','switch','status_led' | ForEach-Object {
        Join-Path $root "METR4810_Team2_code/Core/Src/$_.c"
    })
$objects = @()
Push-Location $output
try {
foreach ($file in $inputs) {
    # Keep compiler inputs and outputs in the ignored build directory.
    $copy = Join-Path $output ([IO.Path]::GetFileName($file))
    Copy-Item -LiteralPath $file -Destination $copy
    & "$LccRoot/bin/lcc64.exe" "-DLIMIT_OVERTRAVEL_STEPS=$OvertravelSteps" "-I$LccRoot/include64" "-I$root/tests/host" "-I$root/METR4810_Team2_code/Core/Inc" $copy
    if ($LASTEXITCODE) { throw "Compile failed: $file" }
    $objects += [IO.Path]::ChangeExtension($copy, '.obj')
}
$exe = Join-Path $output 'test_control.exe'
& "$LccRoot/bin/lcclnk64.exe" "-L$($LccRoot.Replace('/','\'))\lib64" -o $exe @objects
if ($LASTEXITCODE) { throw 'Link failed' }
& $exe
if ($LASTEXITCODE) { throw 'Regression test failed' }
} finally { Pop-Location }
