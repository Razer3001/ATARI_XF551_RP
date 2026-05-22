@echo off
setlocal
set DATA_DIR=%~dp0..\ESP32_Master_Bridge\data
echo Limpiando carpeta data para evitar SPIFFS lleno: %DATA_DIR%
if not exist "%DATA_DIR%" (
  echo No existe carpeta data.
  exit /b 1
)
del /q "%DATA_DIR%\*.atr" 2>nul
del /q "%DATA_DIR%\*.ATR" 2>nul
del /q "%DATA_DIR%\*.xex" 2>nul
del /q "%DATA_DIR%\*.XEX" 2>nul
del /q "%DATA_DIR%\index.html" 2>nul
del /q "%DATA_DIR%\index.html.gz" 2>nul
echo OK. data queda solo para JSON/config pequena.
endlocal
