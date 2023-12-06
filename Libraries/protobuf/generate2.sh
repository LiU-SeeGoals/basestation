#!/usr/bin/bash
PARENT_PATH=$(dirname "${BASH_SOURCE[0]}")

cd $PARENT_PATH

cp ../../proto-messages/*.proto .

protoc-c --c_out=. *.proto

for filename in *.proto; do
    mv "${filename%.*}.pb-c.c" ./Src
    mv "${filename%.*}.pb-c.h" ./Inc
done

rm *.proto

