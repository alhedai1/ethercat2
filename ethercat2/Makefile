CXX = g++
CXXFLAGS = -Wall -O2 -std=c++11 -I$(INCLUDE_DIR)
LDFLAGS = -L/path/to/soem/libs -lsoem

SRC_DIR = ./src
OBJ_DIR = ./obj
INCLUDE_DIR = ./include

SRC = $(SRC_DIR)/main.cpp $(SRC_DIR)/utils.cpp
HEADERS = $(SRC_DIR)/Master.hpp $(SRC_DIR)/Motor.hpp $(INCLUDE_DIR)/include.h

OBJ = $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Debugging output to check SRC and OBJ
$(info SRC = $(SRC))
$(info OBJ = $(OBJ))

OUTPUT = program

# Create object directory if it doesn't exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Default rule
all: $(OUTPUT)

# Rule to compile object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS) | $(OBJ_DIR)
	@echo "Compiling $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to link object files into an executable
$(OUTPUT): $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) -o $(OUTPUT)

# Clean rule to remove object files and executable
clean:
	rm -f $(OBJ) $(OUTPUT)

.PHONY: all clean
