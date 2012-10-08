# This is under MIT licence
# Also, I am not at all proud of this makefile, feel free to make better

all: 
	g++ -o smallvcm ./src/smallvcm.cxx -O3 -std=c++0x -fopenmp

old_rng:
	g++ -o smallvcm ./src/smallvcm.cxx -O3 -fopenmp -DLEGACY_RNG

clean:
	rm smallvcm
