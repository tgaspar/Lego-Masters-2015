
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP
DEPS=ev3dev.h ev3pfilter.h
OBJ=ev3dev.o ev3pfilter.cpp
LIBS=-lstdc++ -lm -I/host-rootfs/home/timo/Lego-Masters-2015

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS) $(LIBS)
	


drop_cilinder: $(OBJ) drop_cilinder.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS) -lpthread

.PHONY: all clean

clean:
	rm -f *.o *~ drop_cilinder

all: drop_cilinder

install: all
	rm -f *.o *~ drop_cilinder



