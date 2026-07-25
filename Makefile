CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall

# Target executable
TARGET := main.out

# Main directory to build
DIR ?= controllers/IRC

# Collect all source files
SRC := \
	$(wildcard $(DIR)/*.cpp) \
	$(wildcard observers/NDO/*.cpp)

# Include directories
INCLUDES := \
	-I. \
	-Iobservers/NDO \
	-Icontrollers/SMC

run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: run clean