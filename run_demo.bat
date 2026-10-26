@echo off
REM One terminal: the whole fire event animated round by round.
REM Usage: run_demo.bat [room]   e.g. run_demo.bat ER
cd /d "%~dp0"
if not exist bin\firealert.exe call build.bat
set ROOM=%1
if "%ROOM%"=="" set ROOM=ICU
bin\firealert.exe demo --fire %ROOM%
pause
