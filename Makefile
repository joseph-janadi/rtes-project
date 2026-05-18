INCLUDE_DIRS = 
LIB_DIRS = 
CC = gcc

CDEFS =
CFLAGS = $(INCLUDE_DIRS) $(CDEFS)
ifeq ($(DEBUG), 1)
    CFLAGS += -O0 -g
else
    CFLAGS += -O3
endif
LDFLAGS = -lrt -lpthread

HFILES = 
CFILES = capture.c

SRCS = ${HFILES} ${CFILES}
OBJS = ${CFILES:.c=.o}
TARGET = capture

all: $(TARGET)

clean:
	-rm -f *.o *.d *.ppm *.pgm
	-rm -f $(TARGET)

capture: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
