all:
	g++ -Wall -D__LINUX_ALSA__ -o lsdj_mo lsdj_mo.cpp RtMidi.cpp -lasound -lpthread -lwiringPi

run: all
	./lsdj_mo
