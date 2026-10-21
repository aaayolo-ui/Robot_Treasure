param(
    [string]$Port = 'COM13',
    [ValidateRange(1200, 1000000)]
    [int]$Baud = 115200
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new()
$serial.PortName = $Port
$serial.BaudRate = $Baud
$serial.DataBits = 8
$serial.Parity = [System.IO.Ports.Parity]::None
$serial.StopBits = [System.IO.Ports.StopBits]::One
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 500
$serial.NewLine = "`n"
$bytes = [System.Collections.Generic.List[byte]]::new()
$latestDistance = $null
$latestConfidence = $null
$lastDisplay = [DateTimeOffset]::MinValue

try {
    $serial.Open()
    Write-Host "Live right laser monitor RX3: $Port / $Baud / 8N1. Press Ctrl+C to stop."
    while ($true) {
        try {
            $line = $serial.ReadLine()
        } catch [System.TimeoutException] {
            continue
        }

        if ($line -notmatch 'HEX=([0-9A-Fa-f ]+)') { continue }
        foreach ($part in ($Matches[1].Trim() -split '\s+')) {
            if ($part.Length -eq 2) {
                $bytes.Add([Convert]::ToByte($part, 16))
            }
        }

        $i = 0
        while ($i -le $bytes.Count - 10) {
            $channel = $bytes[$i]
            if (($channel -ge 49) -and ($channel -le 52) -and
                ($bytes[$i + 1] -eq 35) -and ($bytes[$i + 2] -eq 170) -and
                ($bytes[$i + 3] -eq 5) -and ($bytes[$i + 4] -eq 104)) {
                $sum = 0
                for ($j = 2; $j -le 8; $j++) { $sum += $bytes[$i + $j] }
                if (($sum -band 255) -eq $bytes[$i + 9]) {
                    if ($channel -eq 51) {
                        $latestDistance = [int]$bytes[$i + 6] + ([int]$bytes[$i + 7] * 256)
                        $latestConfidence = [int]$bytes[$i + 8]
                    }
                    $i += 10
                    continue
                }
            }
            $i++
        }
        if ($i -gt 0) { $bytes.RemoveRange(0, $i) }
        if ($bytes.Count -gt 4096) { $bytes.RemoveRange(0, $bytes.Count - 9) }

        $now = [DateTimeOffset]::Now
        if (($null -ne $latestDistance) -and (($now - $lastDisplay).TotalMilliseconds -ge 500)) {
            $centimeters = [math]::Round($latestDistance / 10.0, 1)
            $timestamp = $now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
            Write-Host ('[{0}] RX3 {1} mm ({2} cm) confidence={3}' -f $timestamp, $latestDistance, $centimeters, $latestConfidence)
            $lastDisplay = $now
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
