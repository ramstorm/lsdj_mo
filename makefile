all:
	g++ -Wall -o lsdj_mo lsdj_mo.cpp -lwiringPi

run: all
	./lsdj_mo
