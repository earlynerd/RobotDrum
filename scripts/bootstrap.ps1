param(
  [string]$VenvPath = ".venv",
  [string]$PythonVersion = "3.12",
  [string]$BasePython = "",
  [switch]$Recreate,
  [switch]$SkipPlatformioInstall
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

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

function Resolve-BasePython
{
  param(
    [Parameter(Mandatory = $true)][string]$Version,
    [Parameter(Mandatory = $false)][string]$ExplicitPath = ""
  )

  if (![string]::IsNullOrWhiteSpace($ExplicitPath))
  {
    if (!(Test-Path $ExplicitPath))
    {
      throw "Base python path does not exist: $ExplicitPath"
    }
    return @($ExplicitPath)
  }

  $candidates = @()
  if (Get-Command py -ErrorAction SilentlyContinue)
  {
    $candidates += ,@("py", "-$Version")
    $candidates += ,@("py")
  }

  if (Get-Command python -ErrorAction SilentlyContinue)
  {
    $candidates += ,@("python")
  }

  $programFilesCandidates = @()
  if (Test-Path "C:\Program Files\Python312\python.exe")
  {
    $programFilesCandidates += "C:\Program Files\Python312\python.exe"
  }
  if (Test-Path "C:\Program Files\Python311\python.exe")
  {
    $programFilesCandidates += "C:\Program Files\Python311\python.exe"
  }
  if (Test-Path "C:\Program Files\KiCad\9.0\bin\python.exe")
  {
    $programFilesCandidates += "C:\Program Files\KiCad\9.0\bin\python.exe"
  }

  foreach ($pathCandidate in $programFilesCandidates)
  {
    $candidates += ,@($pathCandidate)
  }

  foreach ($candidate in $candidates)
  {
    & $candidate[0] @($candidate | Select-Object -Skip 1) "--version" *> $null
    if ($LASTEXITCODE -eq 0)
    {
      return $candidate
    }
  }

  throw "Unable to find a usable Python launcher. Install Python $Version for all users or pass -BasePython <absolute path>."
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

function Assert-PythonTempWriteWorks
{
  param(
    [Parameter(Mandatory = $true)][string[]]$PythonCommand
  )

  $probeScript = "import pathlib, tempfile; d = tempfile.mkdtemp(); pathlib.Path(d, 'probe.txt').write_text('ok')"
  & $PythonCommand[0] @($PythonCommand | Select-Object -Skip 1) "-c" $probeScript *> $null
  if ($LASTEXITCODE -ne 0)
  {
    $resolved = $PythonCommand -join " "
    throw "Selected Python failed temp-directory write checks in this sandbox ($resolved). Use a different interpreter path (recommended: Python 3.12 all-users install, or a known-good 3.11 path)."
  }
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot

Ensure-LocalGitConfig -RepoRoot $repoRoot
Ensure-LocalTemp -RepoRoot $repoRoot
Ensure-LocalPlatformioHome -RepoRoot $repoRoot

$basePython = @(Resolve-BasePython -Version $PythonVersion -ExplicitPath $BasePython)
Assert-PythonTempWriteWorks -PythonCommand $basePython
$venvArgs = @("-m", "venv")
if ($Recreate)
{
  $venvArgs += "--clear"
}
$venvArgs += $VenvPath
$venvCommand = @()
$venvCommand += $basePython
$venvCommand += $venvArgs
Invoke-CheckedCommand -Command $venvCommand

$venvPython = Join-Path $repoRoot "$VenvPath\Scripts\python.exe"
if (!(Test-Path $venvPython))
{
  throw "Virtual environment python not found at $venvPython"
}

Invoke-CheckedCommand -Command @($venvPython, "-m", "pip", "install", "--upgrade", "pip")
if (!$SkipPlatformioInstall)
{
  Invoke-CheckedCommand -Command @($venvPython, "-m", "pip", "install", "platformio")
}
else
{
  Write-Host "Skipping platformio install (--SkipPlatformioInstall)."
}

Write-Host ""
Write-Host "Bootstrap complete."
Write-Host "Next: pwsh -File scripts/preflight.ps1"
