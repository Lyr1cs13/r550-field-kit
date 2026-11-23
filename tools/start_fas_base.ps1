param(
    [string]$Python = 'python',
    [string]$Bind = '192.168.169.1',
    [switch]$SetPorts
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if ($SetPorts -or -not (Test-Path -LiteralPath 'base-ports.json')) {
    Get-PnpDevice -PresentOnly -Class Ports | Select-Object -ExpandProperty FriendlyName | Write-Host
    $ws = (Read-Host 'WS63 base receiver COM port').Trim().ToUpperInvariant()
    $c5 = (Read-Host 'C5 base MAIN native USB COM port').Trim().ToUpperInvariant()
    if ($ws -notmatch '^COM[1-9]\d*$' -or $c5 -notmatch '^COM[1-9]\d*$' -or $ws -eq $c5) {
        throw 'Enter two different COM ports.'
    }
    @{ws63=$ws; c5=$c5} | ConvertTo-Json | Set-Content -Encoding ASCII -LiteralPath 'base-ports.json'
}
$port = Get-Content -Raw -LiteralPath 'base-ports.json' | ConvertFrom-Json
& $Python -c "import serial"
if ($LASTEXITCODE -ne 0) { throw 'Install pyserial in this Python environment first.' }
& $Python -c "from pathlib import Path; import secrets; p=Path('base.key'); p.exists() or p.write_text(secrets.token_urlsafe(32))"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$argsForBase = @('--ws63',$port.ws63,'--c5',$port.c5,'--mac-a','D0:CF:13:E4:B6:54',
                '--mac-b','D0:CF:13:E4:BE:10','--key-file','base.key','--data','captures','--bind',$Bind)
& $Python -u fas_base.py @argsForBase
exit $LASTEXITCODE
