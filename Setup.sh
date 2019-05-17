#!/usr/bin/env bash
if [ ! -d "stats" ]; then
        echo "Create dir stats"
        mkdir stats
else
        echo "Directory exists."
fi
chmod 777 ./release/alns_efsmtw_mt
cd release
nohup ./alns_efsmtw_mt > outlog &