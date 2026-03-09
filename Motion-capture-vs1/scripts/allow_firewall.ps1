# Run this in PowerShell AS ADMINISTRATOR on the Windows PC.
# Right-click PowerShell -> "Run as Administrator", then run this script.
#
# These ports match config.py:
#   DISCOVERY_PORT   = 6000
#   DATA_PORT        = 6001
#   FEEDBACK_PORT    = 6002
#   CLOCK_SYNC_PORT  = 6003
#
# The OLD script used 5000/5001 which were WRONG and blocked all traffic.

# Remove any stale rules from old port numbers (safe to ignore errors if they don't exist)
Remove-NetFirewallRule -DisplayName "MoCap Multi-Camera Port 5000" -ErrorAction SilentlyContinue
Remove-NetFirewallRule -DisplayName "MoCap Multi-Camera Port 5001" -ErrorAction SilentlyContinue

# --- Inbound rules (Windows must RECEIVE on these ports) ---
New-NetFirewallRule -DisplayName "MoCap 6000 Discovery  IN"  -Direction Inbound  -LocalPort 6000 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6001 Data       IN"  -Direction Inbound  -LocalPort 6001 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6002 Feedback   IN"  -Direction Inbound  -LocalPort 6002 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6003 ClockSync  IN"  -Direction Inbound  -LocalPort 6003 -Protocol TCP -Action Allow

# --- Outbound rules (Windows must SEND on these ports) ---
New-NetFirewallRule -DisplayName "MoCap 6000 Discovery  OUT" -Direction Outbound -LocalPort 6000 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6001 Data       OUT" -Direction Outbound -LocalPort 6001 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6002 Feedback   OUT" -Direction Outbound -LocalPort 6002 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "MoCap 6003 ClockSync  OUT" -Direction Outbound -LocalPort 6003 -Protocol TCP -Action Allow

Write-Host ""
Write-Host "✅ Firewall rules updated for ports 6000, 6001, 6002, 6003 (inbound + outbound)"
Write-Host ""
Write-Host "Verify with:  netstat -an | findstr '600'"
Write-Host "Then restart: python launch_multi_camera.py --mode server"
