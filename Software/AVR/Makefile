# @author: Jesutofunmi Kupoluyi
# @brief: This is a makefile for compiling and uploading my embedded C programs
# @date: 28 March, 2023

# Mode for source compilation { (c -> C source file), (cpp -> C++ source file), (asm -> Assembly source file) }
MODE = c

# Source files 
SOURCES_C	= 	./src/main.c\
				./src/uart.c\
				./src/commutation.c

# SOURCES_CPP	= 	./src/main.cpp
				

# Include directory
INCLUDES =  -I./includes/

# Target name
TARGET = main

# Microcontroller
MCU = atmega328p

# Fuses - set as appropriate

# For atmega328p -> internal 8MHz RC oscillator
LFUSE = 0xE2 
HFUSE = 0xD9

# Bit clock - for flashing MCU 
BCLK = 10

# Compiler flags
CFLAGS = -Os -g -Wall -Wextra -flto -fuse-linker-plugin -mcall-prologues -mmcu=$(MCU) 
LDFLAGS = -Wl,--gc-sections,-u,vfprintf -lprintf_flt

# Compiler condition switch
ifeq ($(MODE), c)
CC = avr-gcc
else ifeq ($(MODE), cpp)
CC = avr-g++
endif

# Object conditional switch
ifeq ($(MODE), c)
OBJECT = $(SOURCES_C:.c = .o)
else ifeq ($(MODE), cpp)
OBJECT = $(SOURCES_CPP:.cpp = .o)  
endif

# Programmer baud rate
BAUD_RATE = 115200

# Upload mode
PORT = usb

# PORT = /dev/tty.usbserial-A50285BI

# Make all recipe
all: $(TARGET).hex

# Make clean recipe
clean: 
	rm -f *.o *.hex *.obj *.elf

# Binary conversion recipe
%.hex: %.obj
	avr-objcopy -R .eeprom -O ihex $< $@


# Object file generation recipe
%.obj: $(OBJECT)
	$(CC) $(CFLAGS) $(OBJECT) $(LDFLAGS) $(INCLUDES) -o $@

# ELF file generation recipe
%.elf: $(OBJECT)
	$(CC) $(CFLAGS) $(OBJECT) $(LDFLAGS) $(INCLUDES) -o $@

# Make size recipe
size : $(TARGET).elf
	avr-size  $(TARGET).elf

# Make program recipe
program: all $(TARGET).hex size
	avrdude -v -V -p$(MCU) -cusbasp -P$(PORT) -b$(BAUD_RATE) -B $(BCLK) -D -e -Uflash:w:$(TARGET).hex:i
#avrdude -v -V -p$(MCU) -cwiring -P$(PORT) -b$(BAUD_RATE) -D -Uflash:w:$(TARGET).hex

# Make fuse (fuse programming) recipe
fuse:
	avrdude -v -V -p $(MCU) -c usbasp -P $(PORT) -b $(BAUD_RATE) -B $(BCLK) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

# 8MHz int. RC lfuse = 0xE4, hfuse = 0xD9
	

