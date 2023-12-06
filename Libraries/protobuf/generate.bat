@echo off
@setlocal

pushd %~dp0

copy %~dp0..\..\proto-messages\*.proto .

for %%A in (*.proto) do (
	call %~dp0..\..\nanopb\generator\nanopb_generator.bat %%A
	move %%~dpnA.pb.c %~dp0Src
	move %%~dpnA.pb.h %~dp0Inc
)

for %%A in (*.proto) do (
	del %%A
)

popd

