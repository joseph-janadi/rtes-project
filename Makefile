INCLUDE_DIRS = 
LIB_DIRS = 
CC = gcc

CDEFS =
CFLAGS = $(INCLUDE_DIRS) $(CDEFS)
INFO = 1
ASSIGNMENT = 1
ifeq ($(DEBUG), 1)
    CFLAGS += -O0 -g -DDEBUG
else
    CFLAGS += -O3
endif
ifeq ($(INFO), 1)
	CFLAGS += -DINFO
endif
ifeq ($(ASSIGNMENT), 1)
	CFLAGS += -DASSIGNMENT
endif
LDFLAGS = -lrt -lpthread -lm

HFILES = 
CFILES = capture.c

SRCS = ${HFILES} ${CFILES}
OBJS = ${CFILES:.c=.o}
TARGET = capture

all: $(TARGET)

clean:
	-rm -f *.o *.d frames/*.ppm frames/*.pgm frames.mp4
	-rm -f $(TARGET)

capture: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

depend:

video:
	ffmpeg -framerate 4 -pattern_type glob -i "frames/*.ppm" -c:v libx264 -pix_fmt yuv420p frames.mp4

clear-journal:
	sudo journalctl --rotate
	sudo journalctl --vacuum-time=1s

.c.o:
	$(CC) $(CFLAGS) -c $<
