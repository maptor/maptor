@echo off
setlocal enabledelayedexpansion

echo ============================================================================
echo MAPTOR Code Quality Check
echo ============================================================================

set "OVERALL_SUCCESS=1"

echo.
echo [1/4] Running Ruff for unsafe auto-fixes (includes imports)...
python -m ruff check --fix --unsafe-fixes .
if !errorlevel! neq 0 (
    echo WARNING: Ruff unsafe fixes found issues!
    set "OVERALL_SUCCESS=0"
) else (
    echo ✓ Ruff unsafe fixes passed
)

echo.
echo [2/4] Running Ruff for standard auto-fixes...
python -m ruff check --fix .
if !errorlevel! neq 0 (
    echo WARNING: Ruff linting found issues!
    set "OVERALL_SUCCESS=0"
) else (
    echo ✓ Ruff linting passed
)

echo.
echo [3/4] Running Ruff for code formatting...
python -m ruff format .
if !errorlevel! neq 0 (
    echo WARNING: Ruff formatting found issues!
    set "OVERALL_SUCCESS=0"
) else (
    echo ✓ Ruff formatting passed
)

echo.
echo [4/4] Running MyPy for type checking...
python -m mypy --exclude build maptor
if !errorlevel! neq 0 (
    echo WARNING: MyPy type checking found issues!
    set "OVERALL_SUCCESS=0"
) else (
    echo ✓ MyPy type checking passed
)

echo.
echo ============================================================================
if "%OVERALL_SUCCESS%"=="1" (
    echo ✓ All checks passed! Code is ready for commit.
    echo ============================================================================
    exit /b 0
) else (
    echo ✗ Some checks failed. Please fix the issues above.
    echo ============================================================================
    exit /b 1
)
