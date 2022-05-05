@echo off

rem zie compiler output, folder van 'arduino-builder'
set arduino_path=C:\MyRobot\Arduino\Arduino-1.8.19

rem zie compiler output, zoek op -fqbn (vanaf 'fqbn' tot 1e spatie overnemen)
set fqbn=STMicroelectronics:stm32:GenF1:pnum=MAPLEMINI_F103CB,upload_method=swdMethod,xserial=none,usb=none,xusb=FS,opt=osstd,dbg=none,rtlib=nanofp

rem stel configuratie in
"%arduino_path%\arduino_debug" --board %fqbn% --save-prefs
pause