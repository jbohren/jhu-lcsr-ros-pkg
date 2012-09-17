CC = g++
CFLAGS = -Wall
PROG = forcecal

SRCS = forcecal.cpp

LIBS = -lcomedi -lm -lrt

all: $(PROG)

$(PROG):	$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -f $(PROG)
