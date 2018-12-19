CC := g++

EXTRAINCDIRS += -I "../inc"
EXTRALDFLAGS += -L/usr/local/lib -L"../lib"
EXTRALIBS += -lwiringPi -lm -lmosquitto -lpthread -lRTIMULib -lwiringPi `sdl2-config --cflags --libs`

CFLAGS += -O2 -W

MYOBJS =  ../obj/Sonar.o ../obj/nmea.o ../obj/gps.o ../obj/serial.o ../obj/lidar.o ../obj/SendToArm.o ../obj/Mqtt.o
 
.PHONY: all	
all: ../obj/Sonar.o ../obj/nmea.o ../obj/gps.o ../obj/serial.o ../obj/lidar.o ../obj/SendToArm.o ../obj/Mqtt.o bin/main 

.PHONY: clean
clean:
	rm -rf bin/main rm -rf ../obj/*.o


../obj/main.o : main.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/Sonar.o : Sonar.cxx 
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/gps.o : gps.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/serial.o : serial.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/nmea.o : nmea.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/SendToArm.o : SendToArm.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/Mqtt.o : Mqtt.c
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

../obj/lidar.o : Lidar.cxx
	${CC} -c $< ${CFLAGS} -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} 

bin/main : main.c
	${CC} ${CFLAGS} $^ -o $@ ${EXTRALDFLAGS} ${EXTRALIBS} ${EXTRAINCDIRS} ${MYOBJS}

