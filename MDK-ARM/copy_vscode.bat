@echo off
setlocal

REM 获取当前脚本所在目录
set "CURDIR=%~dp0"
REM 计算上一级目录
for %%a in ("%CURDIR%\..") do set "PARENT=%%~fa"

REM 复制 .vscode 文件夹到上一级目录，/E 复制所有子目录和文件，/Y 覆盖
xcopy "%CURDIR%.vscode" "%PARENT%\.vscode" /E /Y /I

echo .vscode copy ok