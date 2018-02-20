# Monitor 2
Multiplatform monitor for e-puck2 robot.

Executable available for Windows and Mac OS X in the wiki: http://www.gctronic.com/doc/index.php/e-puck2#PC_interface

# Build
Project based on Qt 5.10.0

# Deploy
## Windows
Go to the directory of the executable and issue the command:  
`C:\Qt\Qt5.10.0\5.10.0\mingw53_32\bin\windeployqt.exe --compiler-runtime  .`  
Overwrite the file `libstdc++-6.dll` with the one you find in the directory `C:\Qt\Qt5.10.0\5.10.0\mingw53_32\bin`

## Mac OS X
Go to the directory of the executable and issue the command:  
`/Users/$USER/Qt5.10.0/5.10.0/clang_64/bin/macdeployqt EPuckMonitor.app`
