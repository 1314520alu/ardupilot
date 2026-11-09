# Host build of the CS35 ARINC429 codec. waf does not glob src/ or host/.
CC ?= gcc
CFLAGS ?= -Wall -Wextra -Werror -std=c11 -O0 -g -I.
SRCS = src/arinc429_catalog.c src/arinc429_word.c src/arinc429_sched.c src/arinc429_link.c src/arinc429_transport.c host/test_arinc429.c
BIN = build/test_arinc429

.PHONY: all check clean
all: $(BIN)

$(BIN): $(SRCS) arinc429.h arinc429_defs.h
	mkdir -p build
	$(CC) $(CFLAGS) -o $@ $(SRCS)

check: $(BIN)
	./$(BIN)

clean:
	rm -rf build
