
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP
DEPS=ev3dev.h
OBJ=ev3dev.o
LIBS=-lstdc++

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)
	


drop_cilinder: $(OBJ) drop_cilinder.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS) -lpthread

.PHONY: all clean

clean:
	rm -f *.o *~ drop_cilinder

all: drop_cilinder


