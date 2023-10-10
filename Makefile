# Compiler and flags
CC := g++
CFLAGS := -std=c++17 -Wall

# Directories
SRC_DIR := src
BIN_DIR := bin
INC_DIR := $(SRC_DIR)/includes

# Source files and object files
SRCS := $(shell dir /s /b $(SRC_DIR)\*.cc)
OBJS := $(patsubst $(SRC_DIR)/%.cc, $(BIN_DIR)/%.o, $(SRCS))

# Output binary
TARGET := $(BIN_DIR)/apa.exe

# Targets
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	@echo Linking $@..
	$(CC) $(CFLAGS) -o $@ $^

# Compiling
$(BIN_DIR)/%.o: $(SRC_DIR)/%.cc
	@echo Compiling $@..
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -I$(INC_DIR) -c -o $@ $<

# Cleaning
clean:
	@echo Cleaning up..
	@del /q /s $(BIN_DIR)

.PHONY: all clean
