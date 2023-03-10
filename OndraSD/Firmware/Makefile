BASE	= zpu-elf
CC      = $(BASE)-gcc
LD      = $(BASE)-gcc
AS      = $(BASE)-as
CP      = $(BASE)-objcopy
DUMP    = $(BASE)-objdump

ZPUFLEXDIR = ./ZPUFlex
ROMGEN = $(ZPUFLEXDIR)/Firmware/zpuromgen

# we use crt0.s from here
STARTUP_DIR = $(ZPUFLEXDIR)/Firmware/startup

# we fetch ROM prologue / epilogue from here
RTL_DIR = $(ZPUFLEXDIR)/RTL/

# we use printf from here
COMMON_DIR = ./Common_Firmware

BUILD_DIR=zpu_obj

STARTUP_SRC = $(STARTUP_DIR)/crt0.s $(STARTUP_DIR)/premain.s
STARTUP_OBJ = $(patsubst $(STARTUP_DIR)/%.s,$(BUILD_DIR)/%.o,$(STARTUP_SRC))

COMMON_SRC = $(COMMON_DIR)/small_printf.c $(COMMON_DIR)/uart.c $(COMMON_DIR)/uart.h \
	$(COMMON_DIR)/spi.h $(COMMON_DIR)/spi.c $(COMMON_DIR)/swap.c $(COMMON_DIR)/swap.h \
	$(COMMON_DIR)/msCounter.c	
COMMON_OBJ = $(patsubst $(COMMON_DIR)/%.c,$(BUILD_DIR)/%.o,$(COMMON_SRC))


MAIN_PRJ = OndraSD
MAIN_SRC = OndraSD.c minfat.c
MAIN_OBJ = $(COMMON_OBJ) $(patsubst %.c,$(BUILD_DIR)/%.o,$(MAIN_SRC))

LINKMAP  = $(ZPUFLEXDIR)/Firmware/standalone_simple.ld


# Commandline options for each tool.

ZPUOPTS= -mno-poppcrel -mno-pushspadd -mno-callpcrel -mno-byteop -mno-shortop -mno-neg -mno-div -mno-mod # No-neg requires bugfixed toolchain
CFLAGS  = -I. -I$(COMMON_DIR)/ -c -Os $(ZPUOPTS) -DSPI_DEBUG -DPRINTF_HEX_ONLY -DDISABLE_LONG_FILENAMES -DDISABLE_PRINTF 
LFLAGS  = -nostartfiles -Wl,--relax -Os


# Our target.
all: $(BUILD_DIR) $(MAIN_PRJ).bin $(MAIN_PRJ)_ROM.vhd $(MAIN_PRJ).rpt


clean:
	rm -f $(BUILD_DIR)/*.o *.hex *.elf *.map *.lst *.srec $(MAIN_PRJ).rom *~ */*.o *.bin *.rpt


# Convert ELF binary to bin file.
%.bin: %.elf
	$(CP) -O binary $< $@

%.rpt: %.elf
	echo >$@ -n "End of code:\t"
	$(DUMP) -x $< | grep >>$@ _romend
	echo >>$@ -n "Start of BSS:\t"
	$(DUMP) -x $< | grep  >>$@ __bss_start__
	echo >>$@ -n "End of BSS:\t"
	$(DUMP) -x $< | grep >>$@ __bss_end__
	cat $@

$(ROMGEN): $(ZPUFLEXDIR)/Firmware/zpuromgen.c
	gcc -o $(ZPUFLEXDIR)/Firmware/zpuromgen $(ZPUFLEXDIR)/Firmware/zpuromgen.c

%_ROM.vhd: %.bin #$(ROMGEN)
	sed 's/dualportram/$*_ROM/' >$*_ROM.vhd <$(RTL_DIR)/rom_prologue.vhd
	zpuromgen.exe $*.bin >>$*_ROM.vhd
	cat >>$*_ROM.vhd $(RTL_DIR)/rom_epilogue.vhd

# Link - this produces an ELF binary.

$(MAIN_PRJ).elf: $(STARTUP_OBJ) $(MAIN_OBJ)
	$(LD) $(LFLAGS) -T $(LINKMAP) -o $@ $+ $(LIBS)

$(BUILD_DIR)/%.o: %.c Makefile
	$(CC) $(CFLAGS)  -o $@ -c $<
	

$(BUILD_DIR)/%.o: $(COMMON_DIR)/%.c Makefile
	$(CC) $(CFLAGS)  -o $@ -c $<

$(BUILD_DIR)/%.o: %.s
	$(AS) -o $@ $<

$(BUILD_DIR)/%.o: $(STARTUP_DIR)/%.s
	$(AS) -o $@ $<

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

