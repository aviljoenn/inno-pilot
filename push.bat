@echo off
setlocal EnableExtensions DisableDelayedExpansion

if "%~1"=="" (
    echo Usage: %~nx0 Your commit message
    echo Example: %~nx0 Fixed login bug
    exit /b 1
)

rem Build the commit message from all arguments, quoted or unquoted.
set "MSG=%~1"
:build_msg
shift
if "%~1"=="" goto have_msg
set "MSG=%MSG% %~1"
goto build_msg

:have_msg
rem Make sure we are inside a Git repo and move to the repo root.
for /f "delims=" %%I in ('git rev-parse --show-toplevel 2^>nul') do set "REPO_ROOT=%%I"
if not defined REPO_ROOT (
    echo Error: This folder is not inside a Git repository.
    exit /b 1
)

pushd "%REPO_ROOT%" >nul

echo.
echo Repo root: %REPO_ROOT%
echo.
git status --short

echo.
echo Staging all changes...
git add -A
if errorlevel 1 goto fail

echo.
echo Committing...
git commit -m "%MSG%"
if errorlevel 1 goto fail

echo.
echo Pushing...
git push
if errorlevel 1 goto fail

echo.
echo Done.
popd >nul
exit /b 0

:fail
echo.
echo Failed.
popd >nul
exit /b 1
