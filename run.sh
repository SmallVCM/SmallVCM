#!/bin/bash

for iters in 1000
do
for len in 2 3 4 5 6 7 8 9
do
	mkdir nresults_${iters}_${len}
	./Release/smallvcm.exe ${iters} ${len}
	mv *.bmp nresults_${iters}_${len}
	mv report.html nresults_${iters}_${len}
done
done