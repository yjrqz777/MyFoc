
call "setup_mingw.bat"

cd .

if "%1"=="" ("D:\App\MATLAB\R2022a\bin\win64\gmake"  -f vf_ctrl.mk all) else ("D:\App\MATLAB\R2022a\bin\win64\gmake"  -f vf_ctrl.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1