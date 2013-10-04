OBJECTS=manager ifkit phidgetsbclist
CFLAGS=-g -O0 -Wall -I../
LIBS= -lphidget21 -lpthread -ldl

all: $(OBJECTS)

%: %.c
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	-rm *.o $(OBJECTS)
