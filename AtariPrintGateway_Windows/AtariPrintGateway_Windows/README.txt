AtariPrintGateway Windows v29

Ejecutar PowerShell como Administrador:

cd C:\AtariPrintGateway
powershell -ExecutionPolicy Bypass -File .\AtariPrintGateway.ps1 -Prefix "http://+:8080/" -PrinterName "Brother DCP-T720DW"

Firewall, solo una vez:
New-NetFirewallRule -DisplayName "Atari Print Gateway 8080" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8080

Pruebas:
http://localhost:8080/status
http://localhost:8080/test

En el MASTER:
Tipo salida: Gateway HTTP
Gateway HTTP: http://IP_DEL_PC:8080/print

v29 responde HTTP 200 inmediatamente y después manda el trabajo al driver Brother.
