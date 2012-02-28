DEVICE  = atmega1284p
F_CPU   = 20000000	# Hz
FUSE_L  = 0xD7
FUSE_H  = 0x99
FUSE_E  = 0xFC
#AVRDUDE = avrdude -c usbtiny -p $(DEVICE) -B 1
AVRDUDE = avrdude -c usbtiny -p $(DEVICE)

CFLAGS  = -I. -I../../mrbus/src -I../../mrbee/src -DDEBUG_LEVEL=0 -DMRBUS_DISABLE_LOOPBACK
OBJECTS = main.o ../../mrbus/src/mrbus-avr.o ../../mrbus/src/mrbus-crc.o ../../mrbee/src/mrbee.o ../../mrbee/src/xbee.o

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# -E -dM options preprocess only and output results of #define
#COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE) -E -dM

help:
	@echo "make hex ....... build main.hex"
	@echo "make program ... flash fuses and firmware"
	@echo "make fuse ...... flash the fuses"
	@echo "make read ...... read the fuses"
	@echo "make flash ..... flash the firmware"
	@echo "make clean ..... delete objects and hex file"

hex: main.hex

program: flash fuse

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" -a "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid Fuse values."; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

read:
	$(AVRDUDE) -v

# rule for uploading firmware:
flash: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf main.s *.o ../../mrbus/src/*.o ../../mrbee/src/*.o

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.hex

# debugging targets:

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
