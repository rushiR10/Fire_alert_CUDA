@echo off
REM Build on Windows with CUDA 12.x + Visual Studio 2022 Build Tools
setlocal
set VCVARS="C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist %VCVARS% set VCVARS="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
call %VCVARS% >nul
if not exist bin mkdir bin
nvcc -O3 -std=c++17 -Wno-deprecated-gpu-targets ^
  -gencode arch=compute_61,code=sm_61 ^
  -gencode arch=compute_75,code=sm_75 ^
  -gencode arch=compute_86,code=sm_86 ^
  -gencode arch=compute_86,code=compute_86 ^
  -Xcompiler "/O2 /EHsc /utf-8" -lws2_32 -o bin\firealert.exe src\firealert.cu
endlocal
