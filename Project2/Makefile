# for some reason g++ is fine for c, i did try gcc as wellm  there was no diffference
CXX = g++
CC = g++


CXXFLAGS = -Wall -Iclasses -Ifunctions
CFLAGS = -Wall -Ilbfgs

#dirs
SRCDIRS = . classes functions
CSRCDIRS = lbfgs

CPP_SOURCES := $(wildcard $(addsuffix /*.cpp,$(SRCDIRS)))
C_SOURCES := $(wildcard $(addsuffix /*.c,$(CSRCDIRS)))

CPP_OBJECTS := $(CPP_SOURCES:.cpp=.o)
C_OBJECTS := $(C_SOURCES:.c=.o)

OBJECTS := $(CPP_OBJECTS) $(C_OBJECTS)

DEPS := $(CPP_OBJECTS:.o=.d) $(C_OBJECTS:.o=.d)

TARGET = fluid

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

-include $(DEPS)

clean:
	rm -f $(TARGET) $(OBJECTS) $(DEPS)

.PHONY: all clean
