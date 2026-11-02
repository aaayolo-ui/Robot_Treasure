$ErrorActionPreference='Stop'
$appRoot=$PSScriptRoot
Copy-Item -LiteralPath (Join-Path $appRoot 'deps/lib/net462/Microsoft.Web.WebView2.Core.dll') -Destination (Join-Path $appRoot 'assets') -Force
Copy-Item -LiteralPath (Join-Path $appRoot 'deps/lib/net462/Microsoft.Web.WebView2.WinForms.dll') -Destination (Join-Path $appRoot 'assets') -Force
Copy-Item -LiteralPath (Join-Path $appRoot 'deps/runtimes/win-x64/native/WebView2Loader.dll') -Destination (Join-Path $appRoot 'assets') -Force
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip=Join-Path $appRoot 'assets.zip'
if(Test-Path -LiteralPath $zip){Remove-Item -LiteralPath $zip}
$archive=[IO.Compression.ZipFile]::Open($zip,[IO.Compression.ZipArchiveMode]::Create)
try {
    Get-ChildItem -LiteralPath (Join-Path $appRoot 'assets') -File | Where-Object {$_.Extension -in '.py','.html','.css','.js','.json','.dll'} | ForEach-Object {
        [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive,$_.FullName,$_.Name) | Out-Null
    }
} finally {$archive.Dispose()}
$compiler='C:/Windows/Microsoft.NET/Framework64/v4.0.30319/csc.exe'
& $compiler @('/nologo','/target:winexe','/platform:x64',('/win32manifest:'+(Join-Path $appRoot 'app.manifest')),('/win32icon:'+(Join-Path $appRoot 'monitor.ico')),('/out:'+(Join-Path $appRoot '小车串口监控一体助手.exe')),('/resource:'+$zip+',assets.zip'),'/reference:System.Windows.Forms.dll','/reference:System.Drawing.dll','/reference:System.Web.Extensions.dll','/reference:System.Management.dll','/reference:System.IO.Compression.dll',('/reference:'+(Join-Path $appRoot 'assets/Microsoft.Web.WebView2.Core.dll')),('/reference:'+(Join-Path $appRoot 'assets/Microsoft.Web.WebView2.WinForms.dll')),(Join-Path $appRoot 'Program.cs'))
if($LASTEXITCODE -ne 0){throw 'Compilation failed'}
