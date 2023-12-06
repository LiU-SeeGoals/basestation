#!/usr/bin/bash
PARENT_PATH=$(dirname "${BASH_SOURCE[0]}")

cd $PARENT_PATH

cp ../../proto-messages/*.proto .

../../nanopb/generator/nanopb_generator *.proto

for filename in *.proto; do
    mv "${filename%.*}.pb.c" ./Src
    mv "${filename%.*}.pb.h" ./Inc
done

rm *.proto

