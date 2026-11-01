param([string]$RobotIP='192.168.0.100',[string]$VMIP='192.168.80.10',[string]$User='wheeltec')
$ErrorActionPreference='Stop'
$sshDir=Join-Path $env:USERPROFILE '.ssh'
New-Item -ItemType Directory -Force $sshDir | Out-Null
$target=Join-Path $sshDir 'r550-field-kit.conf'
@"
Host r550
    HostName $RobotIP
    User $User
    ConnectTimeout 8
    ServerAliveInterval 15
    ServerAliveCountMax 2

Host r550-vm
    HostName $VMIP
    User $User
    ConnectTimeout 8
    ServerAliveInterval 15
    ServerAliveCountMax 2
"@ | Set-Content -LiteralPath $target -Encoding ascii
$config=Join-Path $sshDir 'config'
$line='Include ~/.ssh/r550-field-kit.conf'
if (Test-Path -LiteralPath $config) {
  $text=Get-Content -LiteralPath $config -Raw
  if (-not $text.Contains($line)) {
    Copy-Item -LiteralPath $config -Destination ($config+'.before-field-kit')
    ($line+"`n"+$text) | Set-Content -LiteralPath $config -Encoding ascii
  }
} else { $line | Set-Content -LiteralPath $config -Encoding ascii }
Write-Output '已配置 ssh r550 与 ssh r550-vm。首次连接按实际主机指纹确认。'
