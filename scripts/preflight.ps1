param(
  [string]$VenvPath = ".venv"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Assert-Exists
{
  param(
    [Parameter(Mandatory = $true)][string]$Path,
    [Parameter(Mandatory = $true)][string]$Message
  )

  if (!(Test-Path $Path))
  {
    throw $Message
  }
}

function Invoke-CheckedCommand
{
  param(
    [Parameter(Mandatory = $true)][string[]]$Command
  )

  Write-Host ("> " + ($Command -join " "))
  & $Command[0] @($Command | Select-Object -Skip 1)
  if ($LASTEXITCODE -ne 0)
  {
    throw "Command failed: $($Command -join ' ')"
  }
}

function Ensure-LocalGitConfig
{
  param(
    [Parameter(Mandatory = $true)][string]$RepoRoot
  )

  $gitConfigPath = Join-Path $RepoRoot ".codex.gitconfig"
  if (!(Test-Path $gitConfigPath))
  {
    Set-Content -Path $gitConfigPath -Value "" -NoNewline
  }

  $env:GIT_CONFIG_GLOBAL = $gitConfigPath

  if (Get-Command git -ErrorAction SilentlyContinue)
  {
    & git config --global --add safe.directory $RepoRoot
  }

  Write-Host "Using GIT_CONFIG_GLOBAL=$gitConfigPath"
}

function Ensure-LocalTemp
{
  param(
    [Parameter(Mandatory = $true)][string]$RepoRoot
  )

  $localTemp = Join-Path $RepoRoot ".codex-tmp"
  if (!(Test-Path $localTemp))
  {
    New-Item -ItemType Directory -Path $localTemp | Out-Null
  }

  $env:TEMP = $localTemp
  $env:TMP = $localTemp
  $env:TMPDIR = $localTemp

  Write-Host "Using TEMP=$localTemp"
}

function Ensure-LocalPlatformioHome
{
  param(
    [Parameter(Mandatory = $true)][string]$RepoRoot
  )

  $pioCoreDir = Join-Path $RepoRoot ".platformio-local"
  if (!(Test-Path $pioCoreDir))
  {
    New-Item -ItemType Directory -Path $pioCoreDir | Out-Null
  }

  $env:PLATFORMIO_CORE_DIR = $pioCoreDir
  Write-Host "Using PLATFORMIO_CORE_DIR=$pioCoreDir"
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot

Ensure-LocalGitConfig -RepoRoot $repoRoot
Ensure-LocalTemp -RepoRoot $repoRoot
Ensure-LocalPlatformioHome -RepoRoot $repoRoot

Assert-Exists -Path (Join-Path $repoRoot "platformio.ini") -Message "Missing platformio.ini"

$venvPython = Join-Path $repoRoot "$VenvPath\Scripts\python.exe"
Assert-Exists -Path $venvPython -Message "Missing virtualenv python at $venvPython. Run scripts/bootstrap.ps1."

Invoke-CheckedCommand -Command @($venvPython, "--version")
Invoke-CheckedCommand -Command @($venvPython, "-m", "platformio", "--version")

Write-Host ""
Write-Host "Preflight OK."
