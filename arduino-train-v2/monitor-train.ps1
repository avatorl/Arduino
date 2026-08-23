param(
  [string] $Port = 'COM8',
  [int] $BaudRate = 115200,
  [int] $DrainMilliseconds = 250
)

if ($BaudRate -le 0) {
  throw 'BaudRate must be greater than zero.'
}

if ($DrainMilliseconds -lt 0) {
  throw 'DrainMilliseconds must not be negative.'
}

$serialPort = $null

try {
  $serialPort = [System.IO.Ports.SerialPort]::new($Port, $BaudRate, 'None', 8, 'One')
  $serialPort.DtrEnable = $false
  $serialPort.RtsEnable = $false
  $serialPort.ReadTimeout = 100

  try {
    $serialPort.Open()
  } catch {
    throw [System.IO.IOException]::new("Failed to open requested serial port '$Port': $($_.Exception.Message)", $_.Exception)
  }

  $serialPort.DiscardInBuffer()
  $serialPort.DtrEnable = $true
  Start-Sleep -Milliseconds $DrainMilliseconds
  $serialPort.DiscardInBuffer()

  Write-Host "Connecting to $Port. Press CTRL-C to exit."
  while ($true) {
    $chunk = $serialPort.ReadExisting()
    if ($chunk.Length -gt 0) {
      [Console]::Write($chunk)
    } else {
      Start-Sleep -Milliseconds 10
    }
  }
} finally {
  if ($null -ne $serialPort) {
    if ($serialPort.IsOpen) {
      $serialPort.Close()
    }
    $serialPort.Dispose()
  }
}
