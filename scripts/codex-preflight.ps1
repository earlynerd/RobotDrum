$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$scriptPath = Join-Path $PSScriptRoot "preflight.ps1"
& $scriptPath @args
exit $LASTEXITCODE
