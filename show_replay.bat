@echo off
REM Browser replay for the presentation: runs one fire event on the GPU,
REM saves it to viewer\replay_data.js and opens viewer\replay.html.
REM Usage: show_replay.bat [room] [people]   e.g. show_replay.bat ER 40
cd /d "%~dp0"
if not exist bin\firealert.exe call build.bat
set ROOM=%1
if "%ROOM%"=="" set ROOM=ICU
set PEOPLE=%2
if "%PEOPLE%"=="" set PEOPLE=40
bin\firealert.exe demo --fire %ROOM% --people %PEOPLE% --delay 0 --no-clear --no-color --export viewer\replay_data.js > nul
if errorlevel 1 (
  echo Could not run the simulation. Check the room name: ICU ER OT Radiology Ward-A NurseStn Ward-B Lab Pharmacy Lobby Admin Canteen
  pause
  exit /b 1
)
echo Replay of a fire in %ROOM% saved. Opening viewer\replay.html ...
start "" "%~dp0viewer\replay.html"
