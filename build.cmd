@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

git submodule update --init --recursive
chdir /d %ROOT_DIR%\external\gflags_airsim
git checkout windows-autobuild
chdir /d %ROOT_DIR%\external\glog_airsim
git checkout windows-autobuild
chdir /d %ROOT_DIR%\external\nlopt_airsim
git checkout windows-autobuild


chdir /d %ROOT_DIR%

REM // Check command line arguments
set "noFullPolyCar=y"
set "buildMode="

REM //check VS version
if "%VisualStudioVersion%" == "" (
    echo(
    echo oh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2019.
    goto :buildfailed_nomsg
)
if "%VisualStudioVersion%" lss "16.0" (
    echo(
    echo Hello there! We just upgraded AirSim to Unreal Engine 4.24 and Visual Studio 2019.
    echo Here are few easy steps for upgrade so everything is new and shiny:
    echo https://github.com/Microsoft/AirSim/blob/master/docs/unreal_upgrade.md
    goto :buildfailed_nomsg
)

if "%1"=="" goto noargs
if "%1"=="--no-full-poly-car" set "noFullPolyCar=y"
if "%1"=="--Debug" set "buildMode=Debug"
if "%1"=="--Release" set "buildMode=Release"
if "%1"=="--RelWithDebInfo" set "buildMode=RelWithDebInfo"

if "%2"=="" goto noargs
if "%2"=="--Debug" set "buildMode=Debug"
if "%2"=="--Release" set "buildMode=Release"
if "%2"=="--RelWithDebInfo" set "buildMode=RelWithDebInfo"

:noargs

:start
chdir /d %ROOT_DIR% 

REM //---------- Check cmake version ----------
CALL check_cmake.bat
if ERRORLEVEL 1 (
  CALL check_cmake.bat
  if ERRORLEVEL 1 (
    echo(
    echo ERROR: cmake was not installed correctly, we tried.
    goto :buildfailed
  )
)

REM //---------- get rpclib ----------
IF NOT EXIST external\rpclib mkdir external\rpclib

set RPC_VERSION_FOLDER=rpclib-2.3.0
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
    REM //leave some blank lines because %powershell% shows download banner at top of console
    ECHO(
    ECHO(   
    ECHO(   
    ECHO *****************************************************************************************
    ECHO Downloading rpclib
    ECHO *****************************************************************************************
	@echo on
	powershell -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://github.com/rpclib/rpclib/archive/v2.3.0.zip -OutFile external\rpclib.zip }"
	@echo off

    REM //remove any previous versions
    rmdir external\rpclib /q /s

	powershell -command "& { Expand-Archive -Path external\rpclib.zip -DestinationPath external\rpclib }"
    del external\rpclib.zip /q
    
    REM //Fail the build if unable to download rpclib
    IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
        ECHO Unable to download rpclib, stopping build
        goto :buildfailed
    )
)

REM //---------- Build gflags ------------
ECHO Starting cmake to build gflags...
IF NOT EXIST external\gflags_airsim\build mkdir external\gflags_airsim\build
cd external\gflags_airsim\build
REM cmake -G"Visual Studio 14 2015 Win64" ..
cmake -G"Visual Studio 16 2019" ..

if "%buildMode%" == "--Debug" (
cmake --build . --config Debug
) else if "%buildMode%" == "--Release" (
cmake --build . --config Release
) else (
cmake --build .
cmake --build . --config Release
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- Build glog --------------
ECHO Starting cmake to build glog...
IF NOT EXIST external\glog_airsim\build mkdir external\glog_airsim\build
cd external\glog_airsim\build
REM cmake -G"Visual Studio 14 2015 Win64" ..
cmake -G"Visual Studio 16 2019" ..

if "%buildMode%" == "--Debug" (
cmake --build . --config Debug
) else if "%buildMode%" == "--Release" (
cmake --build . --config Release
) else (
cmake --build . --config Debug
cmake --build . --config Release
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- Build nlopt -------------
ECHO Starting cmake to build nlopt...
IF NOT EXIST external\nlopt_airsim\build mkdir external\nlopt_airsim\build
cd external\nlopt_airsim\build
REM cmake -G"Visual Studio 14 2015 Win64" ..
cmake -G"Visual Studio 16 2019" ..

if "%buildMode%" == "--Debug" (
cmake --build . --config Debug
) else if "%buildMode%" == "--Release" (
cmake --build . --config Release
) else (
cmake --build .
cmake --build . --config Release
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR%

REM //---------- Build rpclib ------------
ECHO Starting cmake to build rpclib...
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER%\build mkdir external\rpclib\%RPC_VERSION_FOLDER%\build
cd external\rpclib\%RPC_VERSION_FOLDER%\build
cmake -G"Visual Studio 16 2019" ..

if "%buildMode%" == "" (
cmake --build . 
cmake --build . --config Release
) else (
cmake --build . --config %buildMode%
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- copy rpclib binaries and include folder inside AirLib folder ----------
set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%
set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\include %RPCLIB_TARGET_INCLUDE%

if "%buildMode%" == "" (
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Debug %RPCLIB_TARGET_LIB%\Debug
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Release %RPCLIB_TARGET_LIB%\Release
) else (
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\%buildMode% %RPCLIB_TARGET_LIB%\%buildMode%
)

REM //---------- copy gflags binaries and include folder inside AirLib folder ----------
set GFLAGS_TARGET_LIB=AirLib\deps\gflagslib\lib\x64
if NOT exist %GFLAGS_TARGET_LIB% mkdir %GFLAGS_TARGET_LIB%
set GFLAGS_TARGET_INCLUDE=AirLib\deps\gflagslib\include
if NOT exist %GFLAGS_TARGET_INCLUDE% mkdir %GFLAGS_TARGET_INCLUDE%
robocopy /MIR external\gflags_airsim\build\include %GFLAGS_TARGET_INCLUDE%

if "%buildMode%" == "--Debug" (
robocopy /MIR external\gflags_airsim\build\lib\Debug %GFLAGS_TARGET_LIB%\Debug
) else if "%buildMode%" == "--Release" (
robocopy /MIR external\gflags_airsim\build\lib\Release %GFLAGS_TARGET_LIB%\Release
) else (
robocopy /MIR external\gflags_airsim\build\lib\Debug %GFLAGS_TARGET_LIB%\Debug
robocopy /MIR external\gflags_airsim\build\lib\Release %GFLAGS_TARGET_LIB%\Release
)

REM //---------- copy glog binaries and include folder inside AirLib folder ----------
set GLOG_TARGET_LIB=AirLib\deps\gloglib\lib\x64
if NOT exist %GLOG_TARGET_LIB% mkdir %GLOG_TARGET_LIB%
set GLOG_TARGET_INCLUDE=AirLib\deps\gloglib\include\glog
if NOT exist %GLOG_TARGET_INCLUDE% mkdir %GLOG_TARGET_INCLUDE%
robocopy /MIR external\glog_airsim\build\glog %GLOG_TARGET_INCLUDE%

if "%buildMode%" == "--Debug" (
robocopy /MIR external\glog_airsim\build\Debug %GLOG_TARGET_LIB%\Debug
) else if "%buildMode%" == "--Release" (
robocopy /MIR external\glog_airsim\build\Release %GLOG_TARGET_LIB%\Release
) else (
robocopy /MIR external\glog_airsim\build\Debug %GLOG_TARGET_LIB%\Debug
robocopy /MIR external\glog_airsim\build\Release %GLOG_TARGET_LIB%\Release
)

REM //---------- copy nlopt binaries and include folder inside AirLib folder ----------
set NLOPT_TARGET_LIB=AirLib\deps\nloptlib\lib\x64
if NOT exist %NLOPT_TARGET_LIB% mkdir %NLOPT_TARGET_LIB%
set NLOPTLIB_TARGET_INCLUDE=AirLib\deps\nloptlib\include
if NOT exist %NLOPTLIB_TARGET_INCLUDE% mkdir %NLOPTLIB_TARGET_INCLUDE%
robocopy /MIR external\nlopt_airsim\include %NLOPTLIB_TARGET_INCLUDE%

if "%buildMode%" == "--Debug" (
robocopy /MIR external\nlopt_airsim\build\Debug %NLOPT_TARGET_LIB%\Debug
) else if "%buildMode%" == "--Release" (
robocopy /MIR external\nlopt_airsim\build\Release %NLOPT_TARGET_LIB%\Release
) else (
robocopy /MIR external\nlopt_airsim\build\Debug %NLOPT_TARGET_LIB%\Debug
robocopy /MIR external\nlopt_airsim\build\Release %NLOPT_TARGET_LIB%\Release
)

REM //---------- get Eigen library ----------
IF NOT EXIST AirLib\deps mkdir AirLib\deps
IF NOT EXIST AirLib\deps\eigen3 (
    mkdir AirLib\deps\eigen3
    Xcopy /E /I eigen3\Eigen AirLib\deps\eigen3\Eigen
    Xcopy /E /I eigen3\unsupported AirLib\deps\eigen3\unsupported
)
IF NOT EXIST AirLib\deps\eigen3 goto :buildfailed

IF NOT EXIST build_debug (
	mkdir build_debug
	cd build_debug
	echo "Building CMAKE stuff"
	if "%buildMode%" == "--Debug" (
	  cmake ..\cmake -DCMAKE_BUILD_TYPE=Debug -A x64
	)  else if "%buildMode%" == "--Release" (
		cmake ..\cmake -DCMAKE_BUILD_TYPE=Release -A x64
	) else (
		  cmake ..\cmake -DCMAKE_BUILD_TYPE=Debug -A x64
		  cmake ..\cmake -DCMAKE_BUILD_TYPE=Release -A x64
	)
	if ERRORLEVEL 1 goto :buildfailed
	cd ..
)

REM //---------- now we have all dependencies to compile AirSim.sln which will also compile MavLinkCom ----------
cd build_debug
if "%buildMode%" == "--Debug" (
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Debug AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
) else if "%buildMode%" == "--Release" (
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Release AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
) else (
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Debug AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Release AirSim.sln 
if ERRORLEVEL 1 goto :buildfailed
)
cd ..

REM //---------- copy binaries and include for MavLinkCom in deps ----------
set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib\x64
set AIRLIB_TARGET_LIB=AirLib\lib\x64
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%
set MAVLINK_TARGET_INCLUDE=AirLib\deps\MavLinkCom\include
if NOT exist %MAVLINK_TARGET_INCLUDE% mkdir %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\include %MAVLINK_TARGET_INCLUDE%
robocopy /MIR build_debug\output\lib\ %MAVLINK_TARGET_LIB%

robocopy /MIR build_debug\output\lib\ %AIRLIB_TARGET_LIB%


if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp *. /njh /njs /ndl /np
copy /y AirSim.props Unreal\Plugins\AirSim\Source\AirLib

REM //---------- done building ----------
exit /b 0

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2

:buildfailed_nomsg
chdir /d %ROOT_DIR% 
exit /b 1



