@echo off
goto Skip1

******************************************
This is a generated script, do not modify.
******************************************

This script sets the prefences.txt of arduino for this sketch
Suggested name: ArduinoSettings.bat
Call this batchfile with any parameter generates the content
for this file from the current arduino IDE settings.

:Skip1
set sketchdir=C:\GitHub\LiMaPlanck\LiMaPlanck
set sketchname=LiMaPlanck.ino
set fqbn="STMicroelectronics:stm32:GenF1:pnum=MAPLEMINI_F103CB,upload_method=swdMethod,xserial=none,usb=none,xusb=FS,opt=osstd,rtlib=nano"
set Script="C:\RobotLib\Tools\Arduino\ArduinoSettings.py"

if NOT "%1"=="" goto Generate

rem check if settings need to be updated
%Script% %fqbn%
if %ERRORLEVEL%==0 goto done

rem yes => update
C:\MyRobot\Arduino\arduino-1.8.13\arduino_debug --board %fqbn% --save-prefs 2>UpdateSettings.log
if %ERRORLEVEL%==0 goto updated

type UpdateSettings.log
echo *****************************
echo ERROR - something went wrong.
echo *****************************
pause
goto done

:updated
echo IDE settings updated.
pause
goto done

:Generate
%Script%
echo Update content for the batchfile is on the clipboard.

:done
