all:
	gcc -I./ev3dev-c/source/ev3 -O2  -std=gnu99 -W -Wall -c small_stadium_server.c -o small_stadium_server.o
	gcc small_stadium_server.o -Wall -lm -lev3dev-c -lpthread -lbluetooth -o small_stadium_server
	gcc -I./ev3dev-c/source/ev3 -O2  -std=gnu99 -W -Wall -c large_stadium_server.c -o large_stadium_server.o
	gcc large_stadium_server.o -Wall -lm -lev3dev-c -lpthread -lbluetooth -o large_stadium_server
