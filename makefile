all:
	g++ -Wall -D__LINUX_ALSA__ -o lsdj_mo lsdj_mo.cpp RtMidi.cpp -lasound -lpthread -lwiringPi
rt:
	g++ -Wall -D__LINUX_ALSA__ -o lsdj_mo lsdj_rt.cpp RtMidi.cpp -lasound -lpthread -lwiringPi -lrt

run: all
	./lsdj_mo
