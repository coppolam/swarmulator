#!/bin/bash
cd ..
cd logs
mkdir evaluation_time
cd ..
for i in {2 5 10 20 50 100}
do
    ./swarmulator $i
done