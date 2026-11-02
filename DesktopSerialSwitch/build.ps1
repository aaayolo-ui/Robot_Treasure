$ErrorActionPreference = 'Stop'
$switchRoot = $PSScriptRoot
$compiler = 'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe'
& $compiler @('/nologo','/target:exe',('/out:'+(Join-Path $switchRoot 'BuildIcon.exe')),'/reference:System.Drawing.dll',(Join-Path $switchRoot 'BuildIcon.cs'))
if ($LASTEXITCODE -ne 0) { throw 'Icon builder compilation failed' }
& (Join-Path $switchRoot 'BuildIcon.exe') (Join-Path $switchRoot 'monitor.ico')
& $compiler @('/nologo','/target:winexe',('/win32icon:'+(Join-Path $switchRoot 'monitor.ico')),('/out:'+(Join-Path $switchRoot '小车监控开关.exe')),('/resource:'+(Join-Path $switchRoot 'desktop_monitor_host.py')+',desktop_monitor_host.py'),'/reference:System.Windows.Forms.dll','/reference:System.Drawing.dll','/reference:System.Web.Extensions.dll','/reference:System.Management.dll',(Join-Path $switchRoot 'Program.cs'))
if ($LASTEXITCODE -ne 0) { throw 'Desktop switch compilation failed' }
