param(
    [string]$Port = 'COM9',
    [Parameter(Mandatory=$true)]
    [ValidateRange(1200, 1000000)]
    [int]$Baud,
    [string]$OutputDirectory = '',
    [ValidateRange(0, 86400)]
    [int]$Seconds = 0
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    # Resolve after parameter binding: Windows PowerShell may not populate
    # PSScriptRoot while evaluating a parameter's default expression.
    $captureScriptDirectory = $PSScriptRoot
    if ([string]::IsNullOrWhiteSpace($captureScriptDirectory)) {
        $captureScriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
    }
    $OutputDirectory = Join-Path $captureScriptDirectory 'logs'
}
$null = New-Item -ItemType Directory -Path $OutputDirectory -Force
$sessionName = [DateTimeOffset]::Now.ToString('yyyyMMdd-HHmmss-fff')
$rawPath = Join-Path $OutputDirectory ($sessionName + '.log')
$infoPath = Join-Path $OutputDirectory ($sessionName + '.json')
$portReader = [System.IO.Ports.SerialPort]::new()
$portReader.PortName = $Port
$portReader.BaudRate = $Baud
$portReader.DataBits = 8
$portReader.Parity = [System.IO.Ports.Parity]::None
$portReader.StopBits = [System.IO.Ports.StopBits]::One
$portReader.Handshake = [System.IO.Ports.Handshake]::None
$portReader.DtrEnable = $false
$portReader.RtsEnable = $false
$portReader.ReadTimeout = 500
$rawFile = $null
$clock = [Diagnostics.Stopwatch]::new()
$buffer = [byte[]]::new(4096)
$totalBytes = 0L
$startUtc = [DateTimeOffset]::UtcNow.ToString('o')
$lastError = $null

try {
    # Receive only: this program never writes commands or baud probes to the car.
    $portReader.Open()
    $rawFile = [System.IO.File]::Open($rawPath, [IO.FileMode]::CreateNew,
                                    [IO.FileAccess]::Write, [IO.FileShare]::Read)
    $clock.Start()
    Write-Host "Recording $Port at $Baud 8N1. Raw log: $rawPath"
    Write-Host 'Use Ctrl+C to stop. Timing analysis uses timestamps generated on the car.'
    while (($Seconds -eq 0) -or ($clock.Elapsed.TotalSeconds -lt $Seconds)) {
        try {
            $count = $portReader.Read($buffer, 0, $buffer.Length)
        } catch [System.TimeoutException] {
            continue
        }
        if ($count -gt 0) {
            $rawFile.Write($buffer, 0, $count)
            $rawFile.Flush()
            $totalBytes += $count
        }
    }
} catch {
    $lastError = $_.Exception.Message
    throw
} finally {
    $clock.Stop()
    if ($null -ne $rawFile) { $rawFile.Dispose() }
    if ($portReader.IsOpen) { $portReader.Close() }
    $portReader.Dispose()
    [ordered]@{
        port = $Port
        baud = $Baud
        format = '8N1'
        raw_log = $rawPath
        started_utc = $startUtc
        ended_utc = [DateTimeOffset]::UtcNow.ToString('o')
        received_bytes = $totalBytes
        capture_elapsed_s = $clock.Elapsed.TotalSeconds
        error = $lastError
        note = 'Host elapsed time is transport metadata, never vehicle measurement time.'
    } | ConvertTo-Json | Set-Content -LiteralPath $infoPath -Encoding UTF8
    Write-Host "Saved $totalBytes bytes. Metadata: $infoPath"
}
