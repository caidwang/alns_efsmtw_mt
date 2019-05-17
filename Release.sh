#!/usr/bin/env bash
if [ ! -d "Release" ]; then
        echo "create release"
        mkdir Release
        mkdir Release/release
else
        echo "release exists."
fi
cp -ur data Release
cp -ur results Release
cp -ur feasibleResults Release
cp -ur lib Release
cp cmake-build-debug/alns_efsmtw_mt Release/release
cp Setup.sh Release
# cp param.xml Release
echo "Release complete."
