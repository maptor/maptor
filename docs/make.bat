@ECHO OFF

pushd %~dp0

REM Command file for Sphinx documentation

if "%SPHINXBUILD%" == "" (
	set SPHINXBUILD=sphinx-build
)
set SOURCEDIR=source
set BUILDDIR=_build

if "%1" == "" goto help
if "%1" == "clean" goto clean
if "%1" == "html" goto html
if "%1" == "api" goto api
if "%1" == "examples" goto examples

:help
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
goto end

:clean
echo Cleaning documentation...
%SPHINXBUILD% -M clean %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
if exist %SOURCEDIR%\api rmdir /s /q %SOURCEDIR%\api
if exist %SOURCEDIR%\examples rmdir /s /q %SOURCEDIR%\examples
echo Cleaned all build files
goto end

:api
echo Generating API documentation...
cd %SOURCEDIR% && python generate_api_docs.py
goto end

:examples
echo Generating examples documentation...
cd %SOURCEDIR% && python generate_examples_docs.py
goto end

:html
echo Generating API documentation...
cd %SOURCEDIR% && python generate_api_docs.py
cd ..
echo Generating examples documentation...
cd %SOURCEDIR% && python generate_examples_docs.py
cd ..
echo Building HTML documentation...
%SPHINXBUILD% -M html %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
echo Documentation built in _build/html/
goto end

:end
popd
