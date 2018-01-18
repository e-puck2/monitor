# Monitor 2
Multiplatform monitor for e-puck2 robot.

Executable for Windows available here: http://projects.gctronic.com/epuck2/monitor_win.zip  
Executable for Mac OS X available here: http://projects.gctronic.com/epuck2/monitor_mac.zip

# Build
Project based on Qt 5.10.0

# Deploy
## Windows
Go to the directory of the executable and issue the command:  
`C:\Qt\Qt5.10.0\5.10.0\mingw53_32\bin\windeployqt.exe --compiler-runtime  .`  
Overwrite the file `libstdc++-6.dll` with the one you find in the directory `C:\Qt\Qt5.10.0\5.10.0\mingw53_32\bin`

## Mac OS X
Go to the directory of the executable and issue the command:  
`/Users/stefano/Qt5.10.0/5.10.0/clang_64/bin/macdeployqt EPuckMonitor.app`
