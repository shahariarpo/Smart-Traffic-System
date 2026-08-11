# ==============================================================================
# Makefile -- Smart Traffic Signal & Ambulance Priority System
#
# Targets:
#   make            build the release binary (bin/traffic_system)
#   make debug      build with -g -fsanitize=address,undefined for
#                   catching memory errors / UB during development
#   make clean      remove build artifacts
#   make run        build (release) and immediately run it
# ==============================================================================

CC       := gcc
STD      := -std=c11
WARN     := -Wall -Wextra -Wpedantic
OPT      := -O2
SRC      := graph.c dijkstra.c minheap.c signal.c main.c
OBJDIR   := build
BINDIR   := bin
TARGET   := $(BINDIR)/traffic_system
OBJ      := $(SRC:%.c=$(OBJDIR)/%.o)
DEPS     := graph.h dijkstra.h minheap.h signal.h

.PHONY: all debug clean run

all: $(TARGET)

$(TARGET): $(OBJ) | $(BINDIR)
	$(CC) $(STD) $(WARN) $(OPT) -o $@ $(OBJ)

$(OBJDIR)/%.o: %.c $(DEPS) | $(OBJDIR)
	$(CC) $(STD) $(WARN) $(OPT) -c $< -o $@

$(OBJDIR):
	if not exist $(OBJDIR) mkdir $(OBJDIR)

$(BINDIR):
	if not exist $(BINDIR) mkdir $(BINDIR)

debug: CFLAGS_DEBUG := -g -fsanitize=address,undefined -fno-omit-frame-pointer
debug: $(BINDIR)
	$(CC) $(STD) $(WARN) $(CFLAGS_DEBUG) -o $(BINDIR)/traffic_system_debug $(SRC)

run: all
	./$(TARGET)

clean:
	if exist $(OBJDIR) rmdir /s /q $(OBJDIR)
	if exist $(BINDIR) rmdir /s /q $(BINDIR)
