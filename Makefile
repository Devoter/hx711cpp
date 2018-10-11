CC=g++
override CFLAGS += -std=c++17 -c -Wall
LDFLAGS += -lwiringPi
SOURCES=string_to_double.cpp double_to_string.cpp hx711.cpp main.cpp
OBJECTS=$(SOURCES:.cpp=.o)
	EXECUTABLE=hx711

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
		$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
		$(CC) $(CFLAGS) $< -o $@

clean:
		rm -fr $(OBJECTS) $(EXECUTABLE)

