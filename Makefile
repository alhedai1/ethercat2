CXX=aarch64-linux-gnu-g++
CXXFLAGS = -Wall -O2 -std=c++11 -I/usr/local/include -I$(INCLUDE_DIR)
LDFLAGS = -L/usr/local/lib -lsoem

SRC_DIR = ./src
OBJ_DIR = ./obj
INCLUDE_DIR = ./include

SRC = $(SRC_DIR)/main.cpp $(SRC_DIR)/utils.cpp
HEADERS = $(SRC_DIR)/Master.hpp $(SRC_DIR)/Motor.hpp $(INCLUDE_DIR)/include.h

OBJ = $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

$(info SRC = $(SRC))
$(info OBJ = $(OBJ))

OUTPUT = main

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

all: $(OUTPUT)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS) | $(OBJ_DIR)
	@echo "Compiling $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OUTPUT): $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) -o $(OUTPUT)

clean:
	rm -f $(OBJ) $(OUTPUT)

.PHONY: all clean
