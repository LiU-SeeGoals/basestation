@echo off
@setlocal

pushd proto

for %%A in (*.proto) do (
	call %~dp0..\..\..\nanopb\generator\nanopb_generator.bat %%A
	move %%~dpnA.pb.c %~dp0..\Src
	move %%~dpnA.pb.h %~dp0..\Inc
)

popd