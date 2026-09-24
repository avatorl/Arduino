# ==============================================================================
# ENVIRONMENT SHORTCUTS
# ==============================================================================
# Map the 'code' command to VS Code Insiders.
function code {
    & "$env:LocalAppData\Programs\Microsoft VS Code Insiders\bin\code-insiders.cmd" @args
}

# ==============================================================================
# ARDUINO CLI CORE HELPERS
# ==============================================================================

# 1. Maps board shortnames to full Arduino FQBN strings
function Get-ArduinoFqbn {
    param([ValidateSet('uno', 'nano')][string]$Type)

    return @{
        uno  = 'arduino:avr:uno'
        nano = 'arduino:avr:nano'
    }[$Type]
}

# 2. Auto-detects the COM port via arduino-cli JSON output
function Get-ArduinoPort {
    param([string]$Fqbn)

    try {
        $boardsJson = arduino-cli board list --format json | ConvertFrom-Json -ErrorAction Stop
    } catch {
        Write-Error "Could not read Arduino board information: $_"
        return
    }

    $detectedPorts = @(
        $boardsJson.detected_ports |
            Where-Object { $_.matching_boards.fqbn -contains $Fqbn } |
            ForEach-Object { $_.port.address } |
            Where-Object { $_ }
    )

    if ($detectedPorts.Count -eq 0) {
        Write-Error "Could not auto-detect a connected device for: $Fqbn"
        return
    }

    if ($detectedPorts.Count -gt 1) {
        Write-Host "Multiple devices match '$Fqbn':" -ForegroundColor Yellow
        for ($index = 0; $index -lt $detectedPorts.Count; $index++) {
            Write-Host "  $($index + 1). $($detectedPorts[$index])"
        }

        do {
            $selection = Read-Host "Choose a port [1-$($detectedPorts.Count)] or Q to cancel"
            if ($selection -eq 'q') { return }

            [int]$selectedIndex = 0
            $isValidSelection = [int]::TryParse($selection, [ref]$selectedIndex) -and
                $selectedIndex -ge 1 -and $selectedIndex -le $detectedPorts.Count
        } while (!$isValidSelection)

        return $detectedPorts[$selectedIndex - 1]
    }

    return $detectedPorts[0]
}

# 3. Dynamic path resolver with a default base directory prefix
function Resolve-ArduinoPath {
    param([string]$Path)
    if ([string]::IsNullOrWhiteSpace($Path)) { return "." }
    if ($Path -match '^[a-zA-Z]:\\') { return $Path }
    return Join-Path "D:\GITHUB\Arduino\" $Path
}

function Invoke-ArduinoUpload {
    param(
        [string]$BoardType,
        [string]$Fqbn,
        [string]$Port,
        [string]$TargetPath
    )

    Write-Host "Uploading to $($BoardType.ToUpperInvariant()) on $Port..." -ForegroundColor DarkCyan
    arduino-cli upload -p $Port --fqbn $Fqbn $TargetPath | Out-Host

    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Upload successful: uploaded to $($BoardType.ToUpperInvariant()) on $Port." -ForegroundColor Green
        return $true
    }

    Write-Error "Upload failed for $($BoardType.ToUpperInvariant()) on $Port (exit code $LASTEXITCODE)."
    return $false
}

# ==============================================================================
# USER-FACING FUNCTIONS
# ==============================================================================

function arduino {
    param(
        [Parameter(Position=0)][ValidateSet('uno', 'nano', 'help', 'compile', 'upload', 'monitor')][string]$BoardType,
        [Parameter(Position=1, ValueFromRemainingArguments=$true)][ValidateSet('compile', 'upload', 'monitor')][string[]]$Action,
        [Alias('p')][string]$Path,
        [Alias('b')][int]$Baud = 115200,
        [Alias('h')][switch]$Help
    )

        if ($Help -or $BoardType -eq 'help') {
        Write-Host @'
Usage:
  arduino <uno|nano> [compile] [upload] [monitor] [-p <path>] [-b <baud>]

Examples:
  arduino uno
  arduino uno compile
  arduino nano upload monitor -p Blink -b 9600

With no actions, arduino compiles, uploads, then opens the serial monitor.
Upload always compiles first.
Default Baud rate is 115200.
'@
        return
    }

    $selectedBoardType = $BoardType
    $selectedActions = $Action

    if ($selectedBoardType -in 'compile', 'upload', 'monitor') {
        $selectedActions = @($selectedBoardType) + @($selectedActions)
        $selectedBoardType = $null
    }

    while (!$selectedBoardType -or $selectedBoardType -notin 'uno', 'nano') {
        $selectedBoardType = (Read-Host 'Choose board (uno/nano)').ToLowerInvariant()
    }

    if ($selectedActions.Count -eq 0) {
        $selectedActions = 'compile', 'upload', 'monitor'
    }

    if ($selectedActions -contains 'upload' -and $selectedActions -notcontains 'compile') {
        $selectedActions = @('compile') + $selectedActions
    }

    $fqbn = Get-ArduinoFqbn $selectedBoardType
    $targetPath = Resolve-ArduinoPath $Path
    $boardName = $selectedBoardType.ToUpperInvariant()

    if ($selectedActions -contains 'compile') {
        Write-Host "Compiling $boardName at path: $targetPath" -ForegroundColor Cyan
        arduino-cli compile --fqbn $fqbn $targetPath
        if ($LASTEXITCODE -ne 0) {
            Write-Error "Compilation failed (exit code $LASTEXITCODE)."
            return
        }

        Write-Host "✓ Compiled successfully for $boardName." -ForegroundColor Green
    }

    if ($selectedActions -contains 'upload' -or $selectedActions -contains 'monitor') {
        Write-Host "Scanning for connected $boardName" -ForegroundColor DarkCyan
        $port = Get-ArduinoPort $fqbn
        if (!$port) { return }

        Write-Host "✓ Found connected $boardName board on $port." -ForegroundColor Green
    }

    if ($selectedActions -contains 'upload') {
        if (-not (Invoke-ArduinoUpload $selectedBoardType $fqbn $port $targetPath)) { return }
    }

    if ($selectedActions -contains 'monitor') {
        Write-Host "Opening Serial Monitor on $port ($Baud baud)" -ForegroundColor Magenta
        arduino-cli monitor -p $port --config "baudrate=$Baud"
    }
}

# ==============================================================================
# TAB AUTOMATION DEFINITIONS (AUTOCOMPLETE)
# ==============================================================================
$ArduinoBoardsList = @('uno', 'nano')
$ArduinoBoardCompleter = {
    param($commandName, $parameterName, $wordToComplete, $commandAst, $fakeBoundParameters)
    $ArduinoBoardsList | Where-Object { $_ -like "$wordToComplete*" }
}.GetNewClosure()

Register-ArgumentCompleter -CommandName 'arduino' -ParameterName 'BoardType' -ScriptBlock $ArduinoBoardCompleter
