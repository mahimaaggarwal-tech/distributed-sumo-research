# Override with `make CXX=mpicxx` if your MPI compiler wrapper has another name.
CXX = mpicxx
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wpedantic

TARGET := shared_vehicle_state_demo
SOURCE := shared_vehicle_state.cpp

.PHONY: all clean
all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(SOURCE) -o $(TARGET)

clean:
	rm -f $(TARGET)
