# For testing resetting an Arduino via RTS pin using PowerShell
# Simulates a pulse - pulls RTS LOW for a specified duration and then releases it.

# --- Configuration ---
$portName = "COM3" # Adjust to your COM port
$baudRate = 115200 # Adjust to your baud rate
$pulseDurationMs = 100 # Milliseconds for the LOW pulse

Try {
    # Create a new SerialPort object
    $port = New-Object System.IO.Ports.SerialPort($portName, $baudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)

    # Open the port
    $port.Open()
    Write-Host "Serial port $portName opened successfully."

    # Set RTS initially HIGH (inactive reset)
    $port.RtsEnable = $true
    Write-Host "RTS set to HIGH (inactive)."
    Start-Sleep -Milliseconds 300

    # --- The Reset Pulse ---
    Write-Host "Pulsing RTS LOW for $($pulseDurationMs)ms..."
    $port.RtsEnable = $false # Pull RTS LOW (active reset)
    Start-Sleep -Milliseconds $pulseDurationMs # Keep it low for the pulse duration
    $port.RtsEnable = $true # Release RTS, let it go HIGH
    Write-Host "RTS set back to HIGH (inactive)."
    # --- End of Pulse ---

    Start-Sleep -Milliseconds 500 # Give Arduino time to boot up

    # Close the port
    $port.Close()
    Write-Host "Serial port closed."
}
Catch [System.IO.IOException] {
    Write-Error "Error opening or communicating with serial port: $($_.Exception.Message)"
    Write-Host "Please check the port name and ensure no other program is using it."
}
Catch {
    Write-Error "An unexpected error occurred: $($_.Exception.Message)"
}