param([string]$ToolchainBin, [string]$ProjectPath)
$ErrorActionPreference = 'Stop'
if (!$ProjectPath) { $ProjectPath = Join-Path $PSScriptRoot '../METR4810_Team2_code' }
$ProjectPath = (Resolve-Path -LiteralPath $ProjectPath).Path
if (!$ToolchainBin) {
    $compiler = Get-ChildItem 'C:/ST' -Filter arm-none-eabi-gcc.exe -Recurse | Select-Object -First 1
    if (!$compiler) { throw 'Pass -ToolchainBin with the ARM GCC bin directory.' }
    $ToolchainBin = $compiler.DirectoryName
}
$gcc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$output = Join-Path $ProjectPath 'build'
New-Item -ItemType Directory -Force -Path $output | Out-Null
$flags = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
    '-DUSE_HAL_DRIVER','-DSTM32G431xx','-DDEBUG','-O0','-g3','-ffunction-sections','-fdata-sections',
    '-Wall','-Wextra','--specs=nano.specs')
foreach ($inc in @('Core/Inc','Drivers/STM32G4xx_HAL_Driver/Inc',
        'Drivers/STM32G4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include')) {
    $flags += '-I' + (Join-Path $ProjectPath $inc)
}
$sources = @(Get-ChildItem "$ProjectPath/Core/Src/*.c") +
           @(Get-ChildItem "$ProjectPath/Drivers/STM32G4xx_HAL_Driver/Src/*.c") +
           @(Get-ChildItem "$ProjectPath/Core/Startup/*.s")
$objects = @()
foreach ($source in $sources) {
    $object = Join-Path $output ($source.BaseName + '.o')
    & $gcc @flags -c $source.FullName -o $object
    if ($LASTEXITCODE) { throw "Compile failed: $($source.Name)" }
    $objects += $object
}
$elf = Join-Path $output 'METR4810_Team2_code.elf'
& $gcc @flags @objects "-T$ProjectPath/STM32G431RBTX_FLASH.ld" --specs=nosys.specs -static `
    '-Wl,--gc-sections' "-Wl,-Map=$output/METR4810_Team2_code.map" '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o $elf
if ($LASTEXITCODE) { throw 'Link failed' }
& (Join-Path $ToolchainBin 'arm-none-eabi-size.exe') $elf
& (Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $output 'METR4810_Team2_code.hex')
if ($LASTEXITCODE) { throw 'HEX conversion failed' }
Write-Output "Built $elf"
