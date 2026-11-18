param(
    [Parameter(Mandatory)][string]$InterfaceAlias,
    [Parameter(Mandatory)][string]$Ssid,
    [Parameter(Mandatory)][ValidatePattern('^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$')][string]$Bssid,
    [Parameter(Mandatory)][string]$KnownHosts,
    [Parameter(Mandatory)][string]$HostKeyAlias,
    [string]$Vm = 'r550-vm',
    [int]$RelayPort = 18892,
    [int]$VehiclePort = 8892
)

# The second subnet has the same IPv4 addresses as the first. Bind each car SSH
# connection to the second adapter's source address and check its association.
$ErrorActionPreference = 'Stop'
function Start-Tunnel([string[]]$Arguments) {
    $options = [Diagnostics.ProcessStartInfo]::new((Get-Command ssh.exe).Source)
    $options.UseShellExecute = $false
    $options.CreateNoWindow = $true
    $options.RedirectStandardInput = $true
    foreach ($argument in $Arguments) { $options.ArgumentList.Add($argument) }
    [Diagnostics.Process]::Start($options)
}

function Get-SecondAddress {
    $blocks = (netsh wlan show interfaces | Out-String) -split '(?m)^[ \t]*Name[ \t]+:'
    $aliasPattern = '^\s+' + [regex]::Escape($InterfaceAlias) + '\s'
    $ap = $blocks | Where-Object { $_ -match $aliasPattern }
    if (-not $ap -or $ap -notmatch ('(?m)^\s*SSID\s*:\s*' + [regex]::Escape($Ssid) + '\s*$') -or $ap -notmatch $Bssid) { return }
    (Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.AddressState -eq 'Preferred' -and $_.IPAddress -like '192.168.0.*' } |
        Select-Object -First 1).IPAddress
}

$local = Get-SecondAddress
if (-not $local) { throw 'The configured adapter must be connected to the second robot' }
$portPattern = '^\s*TCP\s+127\.0\.0\.1:' + $RelayPort + '\s+0\.0\.0\.0:0\s+'
if (netstat -ano -p tcp | Select-String $portPattern) { throw 'The local relay port is already in use' }
$common = @('-N','-T','-oBatchMode=yes','-oConnectTimeout=5','-oExitOnForwardFailure=yes',
            '-oServerAliveInterval=5','-oServerAliveCountMax=3')
$toVehicle = $toVm = $null
try {
    $toVehicle = Start-Tunnel ($common + @('-b',$local,"-oHostKeyAlias=$HostKeyAlias","-oUserKnownHostsFile=$KnownHosts",'-oStrictHostKeyChecking=yes',
                  '-L',"127.0.0.1:$($RelayPort):127.0.0.1:$VehiclePort",'wheeltec@192.168.0.100'))
    Start-Sleep -Seconds 4
    if ($toVehicle.HasExited -or -not (netstat -ano -p tcp | Select-String ($portPattern + '\S+\s+' + $toVehicle.Id + '\s*$'))) {
        throw 'The second robot SSH forward did not start'
    }
    $toVm = Start-Tunnel ($common + @('-R',"127.0.0.1:$($RelayPort):127.0.0.1:$RelayPort",$Vm))
    Start-Sleep -Seconds 3
    & ssh.exe '-oBatchMode=yes' '-oConnectTimeout=4' $Vm "ss -lnt | grep '127.0.0.1:$RelayPort'"
    if ($LASTEXITCODE -ne 0 -or $toVm.HasExited) { throw 'The VM reverse SSH forward did not start' }
    [pscustomobject]@{VehicleSshProcess=$toVehicle.Id;VmSshProcess=$toVm.Id;RelayPort=$RelayPort}
} catch {
    foreach ($process in @($toVm,$toVehicle)) {
        if ($process -and -not $process.HasExited) { $process.Kill() }
    }
    throw
}
