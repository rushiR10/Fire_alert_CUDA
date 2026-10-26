@echo off
REM Three windows: alert server, phones, fire sensor.
REM Usage: run_all.bat [room]   e.g. run_all.bat ER
cd /d "%~dp0"
if not exist bin\firealert.exe call build.bat
set ROOM=%1
if "%ROOM%"=="" set ROOM=ICU
start "1 - ALERT SERVER (GPU)" cmd /k bin\firealert.exe server
timeout /t 2 /nobreak >nul
start "2 - PHONES D01 P09 V04 N03" cmd /k bin\firealert.exe phone --id D01,P09,V04,N03
timeout /t 2 /nobreak >nul
start "3 - FIRE SENSOR" cmd /k bin\firealert.exe sensor --room %ROOM%
