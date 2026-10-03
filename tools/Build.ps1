[CmdletBinding()]
param(
    [string]$BuildDirectory = 'build-release',
    [switch]$RunTests
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not [System.IO.Path]::IsPathRooted($BuildDirectory)) {
    $BuildDirectory = Join-Path $projectRoot $BuildDirectory
}
$cachePath = Join-Path $BuildDirectory 'CMakeCache.txt'
if (-not (Test-Path -LiteralPath $cachePath)) {
    throw "Configure this build directory with CMake or CLion first: $BuildDirectory"
}
$cache = @{}
foreach ($row in Get-Content -LiteralPath $cachePath) {
    if ($row -match '^([^#/:][^:]*):[^=]+=(.*)$') {
        $cache[$Matches[1]] = $Matches[2]
    }
}
foreach ($key in @('CMAKE_COMMAND', 'CMAKE_CXX_COMPILER')) {
    if (-not $cache[$key] -or -not (Test-Path -LiteralPath $cache[$key])) {
        throw "Missing tool in CMake cache: $key. Reconfigure the build in CLion."
    }
}
if ($RunTests -and (-not $cache['CMAKE_CTEST_COMMAND'] -or
        -not (Test-Path -LiteralPath $cache['CMAKE_CTEST_COMMAND']))) {
    throw 'CMake cache does not contain an available CTest executable.'
}

$originalPath = $env:PATH
try {
    # GCC launches cc1plus from libexec; its runtime DLLs live in bin.
    $compilerDirectory = Split-Path $cache['CMAKE_CXX_COMPILER'] -Parent
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $originalPath
    & $cache['CMAKE_COMMAND'] --build $BuildDirectory --config Release --parallel 2
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }
    if ($RunTests) {
        & $cache['CMAKE_CTEST_COMMAND'] --test-dir $BuildDirectory --output-on-failure -j 1
        if ($LASTEXITCODE -ne 0) {
            throw "Tests failed with exit code $LASTEXITCODE"
        }
    }
} finally {
    $env:PATH = $originalPath
}
