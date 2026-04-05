@echo off
setlocal EnableExtensions DisableDelayedExpansion

if "%~1"=="" (
    echo Usage: %~nx0 "Your commit message"
    echo Example: %~nx0 "fix: prevent false motor activation in HAND mode"
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
echo Current changes:
git status --short

rem Check whether there is anything to do at all.
set "HAS_CHANGES="
for /f "delims=" %%I in ('git status --porcelain') do set "HAS_CHANGES=1"

if not defined HAS_CHANGES (
    echo.
    echo Nothing to commit.
    popd >nul
    exit /b 0
)

echo.
echo Staging all changes...
git add -A
if errorlevel 1 goto fail

echo.
echo Staged changes:
git status --short

echo.
echo Committing...
git commit -m "%MSG%"
if errorlevel 1 goto fail

echo.
echo Pushing...
git push
if errorlevel 1 (
    echo Standard push failed. Trying first-push upstream setup...
    git push -u origin HEAD
    if errorlevel 1 goto fail
)

echo.
echo Done.
popd >nul
exit /b 0

:fail
echo.
echo Failed.
popd >nul
exit /b 1