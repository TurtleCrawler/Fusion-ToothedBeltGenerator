@echo off
echo Installing Toothed Belt Generator plugin bundle for Autodesk Fusion...

set "BUNDLE_DIR=%appdata%\Autodesk\ApplicationPlugins\ToothedBeltGenerator.bundle"
set "CONTENTS_DIR=%BUNDLE_DIR%\Contents"

:: Remove old flat files if they exist from previous install attempt
if exist "%BUNDLE_DIR%" rmdir /S /Q "%BUNDLE_DIR%"

:: Create the .bundle\Contents directory structure
mkdir "%CONTENTS_DIR%"

:: Copy all add-in files into the Contents subfolder
xcopy /E /Y /I "ToothedBeltGenerator" "%CONTENTS_DIR%"

:: Generate the required PackageContents.xml in the .bundle root
(
echo ^<?xml version="1.0" encoding="utf-8"?^>
echo ^<ApplicationPackage SchemaVersion="1.0" ProductType="Application" AutodeskProduct="Fusion" Name="Toothed Belt Generator" Description="Generate and edit parametric GT1.5, GT2, HTD 5M, and HTD 8M timing belts." AppVersion="1.2.0" Author="Mike Cloutier" AppNameSpace="ToothedBeltGenerator" ProductCode="{8A9F4D2B-5C3E-4A1D-9B7F-2E6D1C8A0B3F}" UpgradeCode="{9B0E5F3C-6D4F-4B2E-8C9A-3F7E2D9B1C4A}"^>
echo   ^<Components^>
echo     ^<RuntimeRequirements OS="win64^|mac" Platform="Fusion" /^>
echo     ^<ComponentEntry AppName="ToothedBeltGenerator" Version="1.2.0" ModuleName="./Contents/ToothedBeltGenerator.manifest" /^>
echo   ^</Components^>
echo ^</ApplicationPackage^>
) > "%BUNDLE_DIR%\PackageContents.xml"

echo.
echo Installation complete! Please restart Autodesk Fusion.
pause