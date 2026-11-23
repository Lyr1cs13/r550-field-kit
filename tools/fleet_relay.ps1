param(
    [string]$CarASsid = 'WHEELTEC_RDK_X5_humble',
    [string]$CarBSsid = 'WHEELTEC_RDK_X5_humble_B',
    [string]$CarBHostKey = 'r550-b-1835af',
    [string]$CarBKnownHosts = (Join-Path ([Environment]::GetFolderPath('UserProfile')) '.ssh\known_hosts_r550_b'),
    [string]$CarABindAddress,
    [string]$CarBBindAddress,
    [switch]$StartPanels,
    [switch]$SyncCarAClock,
    [ValidateRange(1024,65529)][int]$RelayPort = 18890,
    [string]$Vm = 'r550-vm',
    [string]$LogPath = (Join-Path $PSScriptRoot '../state/logs/fleet-relay.log')
)

$ErrorActionPreference = 'Stop'

function Report([string]$Message) {
    Add-Content -LiteralPath $LogPath -Encoding utf8 -Value ("{0:yyyy-MM-dd HH:mm:ss} {1}" -f (Get-Date), $Message)
}

function Get-CarAddresses {
    $addresses = @{}
    $adapters = @{}
    foreach ($nic in Get-NetAdapter) {
        $guid = ([guid]$nic.InterfaceGuid).ToString()
        $adapters[$guid] = $nic.ifIndex
    }
    $interfaces = (netsh wlan show interfaces | Out-String) -split '(?m)^[ \t]*GUID[ \t]*:[ \t]*'
    foreach ($block in $interfaces | Select-Object -Skip 1) {
        $guid = ($block -split '\r?\n', 2)[0].Trim().Trim('{}').ToLowerInvariant()
        if (-not $adapters.ContainsKey($guid)) { continue }
        $ssid = [regex]::Match($block, '(?m)^[ \t]*SSID[ \t]+:[ \t]*(?<value>[^\r\n]+)').Groups['value'].Value.Trim()
        if ($ssid -notin @($CarASsid,$CarBSsid)) { continue }
        $ip = (Get-NetIPAddress -InterfaceIndex $adapters[$guid] -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.AddressState -eq 'Preferred' -and $_.IPAddress -like '192.168.0.*' } |
            Select-Object -First 1).IPAddress
        if ($ip -and $addresses.ContainsKey($ssid)) { $addresses[$ssid] = $null }
        elseif ($ip) { $addresses[$ssid] = $ip }
    }
    return $addresses
}

function Resolve-BindAddress([string]$Detected, [string]$Saved) {
    if ($Detected) { return $Detected }
    if ($Saved -and (Get-NetIPAddress -IPAddress $Saved -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object AddressState -eq 'Preferred')) {
        return $Saved
    }
}

function Start-Relay([string[]]$Arguments) {
    $info = [Diagnostics.ProcessStartInfo]::new((Get-Command ssh.exe).Source)
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    foreach ($argValue in $Arguments) { $info.ArgumentList.Add($argValue) }
    return [Diagnostics.Process]::Start($info)
}

function Maintain-Relay($Relay, [string]$Address, [string[]]$Arguments) {
    if ($Relay.Process -and ($Relay.Process.HasExited -or $Relay.Address -ne $Address)) {
        if (-not $Relay.Process.HasExited) { $Relay.Process.Kill() }
        $Relay.Process.Dispose()
        $Relay.Process = $null
        Report "$($Relay.Name) connection closed"
    }
    if ($Address -and -not $Relay.Process) {
        $options = $common
        if ($StartPanels -and $Relay.StartCommand) {
            $options = @($common | Where-Object { $_ -ne '-N' })
            $command = $Relay.StartCommand
            if ($Relay.SendClock) {
                $epoch = ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0).ToString(
                    'F3', [Globalization.CultureInfo]::InvariantCulture)
                $command = $command.Replace('{HOST_UTC}', $epoch)
            }
            $Arguments += $command
        }
        $Relay.Process = Start-Relay ($options + $Arguments)
        $Relay.Address = $Address
        Report "$($Relay.Name) connecting"
    }
}

if ($CarASsid -eq $CarBSsid -or -not (Test-Path -LiteralPath $CarBKnownHosts -PathType Leaf)) {
    throw '需要分别识别两台车的热点并保存二号车的 SSH 主机密钥'
}
$LogPath = [IO.Path]::GetFullPath($LogPath)
[void][IO.Directory]::CreateDirectory((Split-Path -Parent $LogPath))
$common = @('-N','-T','-oBatchMode=yes','-oConnectTimeout=6','-oConnectionAttempts=1',
            '-oExitOnForwardFailure=yes','-oServerAliveInterval=5','-oServerAliveCountMax=4',
            '-oStrictHostKeyChecking=yes')
$a = @{Name='car A'; Process=$null; Address=$null}
$b = @{Name='car B'; Process=$null; Address=$null}
$vmRelay = @{Name='VM'; Process=$null; Address=$null}
$a.StartCommand = 'if cd "$HOME/r550-patrol"; then if ./kit profile select; then ./kit site; if ! curl --noproxy "*" -fs --max-time 2 http://127.0.0.1:8890/api/state >/dev/null; then ./kit nav; fi; fi; fi; exec sleep infinity'
if ($SyncCarAClock) {
    $a.SendClock = $true
    $a.StartCommand = 'if cd "$HOME/r550-patrol" && ./kit profile select; then if ! curl --noproxy "*" -fs --max-time 2 http://127.0.0.1:8890/api/state >/dev/null; then if sudo -n -- /usr/local/sbin/r550-host-clock {HOST_UTC}; then ./kit nav; fi; fi; ./kit site; fi; exec sleep infinity'
}
$b.StartCommand = 'if cd "$HOME/lab_cruise_dual" && ! curl --noproxy "*" -fs --max-time 2 http://127.0.0.1:8892/api/state >/dev/null; then ./kit nav; fi; exec sleep infinity'
$claimed = $false
$portA=$RelayPort
$portB=$RelayPort+2
$siteA=$RelayPort+6
$mutex = [Threading.Mutex]::new($false, "Local\R550FleetRelay-${portA}-${portB}")
try {
    try { $claimed = $mutex.WaitOne(0) }
    catch [Threading.AbandonedMutexException] { $claimed = $true }
    if (-not $claimed) { return }
    Report 'Fleet relay started'
    while ($true) {
        $addresses = Get-CarAddresses
        $ipA = Resolve-BindAddress $addresses[$CarASsid] $CarABindAddress
        $ipB = Resolve-BindAddress $addresses[$CarBSsid] $CarBBindAddress
        Maintain-Relay $a $ipA @('-oCompression=yes','-b',$ipA,'-L',"127.0.0.1:${portA}:127.0.0.1:8890",
                          '-L',"127.0.0.1:${siteA}:127.0.0.1:8896",'r550')
        Maintain-Relay $b $ipB @('-oCompression=yes','-b',$ipB,"-oHostKeyAlias=$CarBHostKey","-oUserKnownHostsFile=$CarBKnownHosts",
                          '-L',"127.0.0.1:${portB}:127.0.0.1:8892",'wheeltec@192.168.0.100')
        Maintain-Relay $vmRelay $Vm @('-R',"127.0.0.1:${portA}:127.0.0.1:${portA}",
                          '-R',"127.0.0.1:${portB}:127.0.0.1:${portB}",'-R',"127.0.0.1:${siteA}:127.0.0.1:${siteA}",$Vm)
        Start-Sleep -Seconds 2
    }
} finally {
    if ($claimed) {
        foreach ($relay in @($a,$b,$vmRelay)) {
            if ($relay.Process -and -not $relay.Process.HasExited) { $relay.Process.Kill() }
        }
        $mutex.ReleaseMutex()
    }
    $mutex.Dispose()
}
