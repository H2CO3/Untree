CXX = xcrun -sdk macosx clang++
LD = $(CXX)

CXFLAGS = -c -std=c++14 -Wall -O2 -flto $(DEBUG) $(shell pkg-config --cflags libpng)
LDFLAGS = -O2 -flto

LIBS = $(shell pkg-config --libs libpng)
OBJECTS = imgproc.o untree.o
TARGET = untree
NDEBUG ?= 0

ifeq ($(NDEBUG),0)
DEBUG = -UNDEBUG
else
DEBUG = -DNDEBUG
endif

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.cc
	$(CXX) $(CXFLAGS) -o $@ $<

clean:
	rm -f $(TARGET) $(OBJECTS)

.phony: all clean
