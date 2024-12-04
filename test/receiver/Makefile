CC = gcc
CFLAGS = -Wall -Wextra

TARGET = softwareUARTTest
SRC = main.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)