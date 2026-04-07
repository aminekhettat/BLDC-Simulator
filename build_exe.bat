@echo off
:: =============================================================================
:: SPINOTOR — Windows executable build script
:: Run this script from the project root directory.
:: =============================================================================

setlocal enabledelayedexpansion

echo.
echo  ==========================================
echo   SPINOTOR — Build executable (Windows)
echo  ==========================================
echo.

:: ---- Check Python ----
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found in PATH. Install Python 3.12+ and retry.
    pause & exit /b 1
)

:: ---- Install / upgrade PyInstaller ----
echo [1/4] Installing PyInstaller...
pip install --upgrade pyinstaller >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Failed to install PyInstaller.
    pause & exit /b 1
)

:: ---- Install UPX (optional, speeds up loading) ----
echo [2/4] Checking UPX (optional compressor)...
where upx >nul 2>&1
if errorlevel 1 (
    echo       UPX not found — build will still succeed, just slightly larger.
) else (
    echo       UPX found.
)

:: ---- Clean previous build artefacts ----
echo [3/4] Cleaning previous build artefacts...
if exist build\SPINOTOR  rmdir /s /q build\SPINOTOR
if exist dist\SPINOTOR   rmdir /s /q dist\SPINOTOR

:: ---- Run PyInstaller ----
echo [4/4] Building SPINOTOR.exe ...
echo.
pyinstaller --noconfirm SPINOTOR.spec

if errorlevel 1 (
    echo.
    echo [ERROR] PyInstaller failed. Check the output above for details.
    pause & exit /b 1
)

echo.
echo  ==========================================
echo   Build successful!
echo   Executable folder: dist\SPINOTOR\
echo   Launch with:       dist\SPINOTOR\SPINOTOR.exe
echo  ==========================================
echo.
pause
endlocal
