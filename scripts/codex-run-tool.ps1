param(
  [Parameter(Mandatory = $true, Position = 0)][string]$Tool,
  [Parameter(ValueFromRemainingArguments = $true)][string[]]$ToolArgs
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

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
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot
Ensure-LocalGitConfig -RepoRoot $repoRoot
Ensure-LocalTemp -RepoRoot $repoRoot
Ensure-LocalPlatformioHome -RepoRoot $repoRoot

$venvScripts = Join-Path $repoRoot ".venv\Scripts"
$venvPython = Join-Path $venvScripts "python.exe"
if (Test-Path $venvScripts)
{
  $env:Path = "$venvScripts;$env:Path"
}

$toolLower = $Tool.ToLowerInvariant()

if (($toolLower -eq "pio" -or $toolLower -eq "platformio") -and (Test-Path $venvPython))
{
  & $venvPython -m platformio @ToolArgs
  exit $LASTEXITCODE
}

$resolved = Get-Command $Tool -ErrorAction SilentlyContinue
if ($null -eq $resolved)
{
  throw "Unable to resolve tool '$Tool'."
}

& $resolved.Source @ToolArgs
exit $LASTEXITCODE
