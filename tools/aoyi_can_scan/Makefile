CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -I/usr/local/include
LDFLAGS ?=
LDLIBS ?= -pthread /usr/local/lib/libserial.a

TARGET := aoyi_can_scan
SRCS := main.cpp Ti5_socketcan.cpp aoyihand.cpp

.PHONY: all clean run scan test

all: $(TARGET)

$(TARGET): $(SRCS) Ti5_socketcan.h aoyihand.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDFLAGS) $(LDLIBS)

scan: $(TARGET)
	sudo ./$(TARGET) scan

test: $(TARGET)
	sudo ./$(TARGET) test

clean:
	rm -f $(TARGET)
