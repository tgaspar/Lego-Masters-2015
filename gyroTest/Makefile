
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP
DEPS=ev3dev.h
OBJ=ev3dev.o
LIBS=-lstdc++

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

gyro-test: $(OBJ) gyro-test.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS) -lpthread

.PHONY: all clean

clean:
	rm -f *.o *~ gyro-test

all: gyro-test
