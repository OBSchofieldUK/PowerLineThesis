echo off
cls
echo Cleaning project... 
mingw32-make clean

echo --------------------------------------------------------------------------------
echo Building project... 
mingw32-make

'echo --------------------------------------------------------------------------------
'echo Building test application...
'cd ../../../Test
'del *.o *.exe
'mingw64-make
