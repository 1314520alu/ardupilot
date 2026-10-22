# Host build only. These sources are not in the vehicle waf glob.
CC ?= gcc
CFLAGS ?= -Wall -Wextra -Werror -std=c11 -O0 -g -I.
SRCS = src/canopen_master.c src/erob_servo.c host/can_bus_stub.c host/main.c
BIN = build/erob_host_demo

.PHONY: all check clean
all: $(BIN)

$(BIN): $(SRCS)
	mkdir -p build
	$(CC) $(CFLAGS) -o $@ $(SRCS)

check: $(BIN)
	./$(BIN)

clean:
	rm -rf build
