#Requires -Version 7.0
[CmdletBinding()]
param([ValidateSet('Release')][string]$Configuration = 'Release')
$ErrorActionPreference = 'Stop'
$env:DOTNET_CLI_TELEMETRY_OPTOUT = '1'
$env:DOTNET_NOLOGO = '1'
$env:DOTNET_CLI_HOME = Join-Path $PSScriptRoot '.dotnet'
$env:DALAMUD_HOME = Join-Path $env:APPDATA 'XIVLauncherCN/addon/Hooks/dev'
$dotnet = Join-Path $env:LOCALAPPDATA 'Microsoft/dotnet/dotnet.exe'
Push-Location $PSScriptRoot
try {
    & $dotnet build Mr.Bean3.csproj -c $Configuration --disable-build-servers
    exit $LASTEXITCODE
} finally { Pop-Location }
