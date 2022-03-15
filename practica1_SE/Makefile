#TOOLCHAIN=~/toolchain/gcc-arm-none-eabi-4_9-2014q4/bin
#PREFIX=$(TOOLCHAIN)/arm-none-eabi-
PREFIX=arm-none-eabi-

ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -O0 -Wall -Werror -D "CPU_MKL46Z256VLL4" $(ARCHFLAGS)

CFLAGS=-I./includes -I./drivers -I./CMSIS -I./board -I./board/src -I./utilities $(COMMONFLAGS)
LDFLAGS_L=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-section,_Map, led_blinky.map,-Tlink.ld
LDFLAGS_H=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-section,_Map, hello_world.map,-Tlink.ld
LDLIBS=

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

TARGET_L= led_blinky
TARGET_H= hello_world
START = startup

SRC_H=$(wildcard board/pin_mux2.c) 
OBJ_H=$(patsubst %.c, %.o, $(SRC_H))

SRC_L=$(wildcard board/pin_mux.c) 
OBJ_L=$(patsubst %.c, %.o, $(SRC_L))

SRC = $(wildcard board/src/*.c  CMSIS/*.c drivers/*.c utilities/*.c)
OBJ_SRC = $(patsubst %.c, %.o, $(SRC))

OBJ_T_L = $(patsubst %.c, %.o, $(TARGET_L).c)
OBJ_T_H = $(patsubst %.c, %.o, $(TARGET_H).c)
OBJ_START = $(patsubst %.c, %.o, $(START).c)

all: build_h size h \ build_l size_l

build_h: elf_h srec_h bin_h
build_l: elf_l srec_l bin_l
elf_h:$(TARGET_H).elf
elf_l: $(TARGET_L).elf 

srec_h: $(TARGET_H).srec
srec_l: $(TARGET_L).srec

bin_h: $(TARGET_H).srec
bin_l: $(TARGET_L).bin

clean:
	$(RM) $(TARGET_L).srec $(TARGET_L).elf $(TARGET_L).bin $(TARGET_L).map $(TARGET_H).srec $(TARGET_H).elf $(TARGET_H).bin $(TARGET_H).map $(OBJ_H) $(OBJ_L) $(OBJ_SRC) $(OBJ_START)

$(TARGET_H).elf: $(OBJ_H) $(OBJ_SRC) $(OBJ_START) $(OBJ_T_H) 
	$(LD) $(LDFLAGS_H) $(OBJ_H) $(OBJ_SRC) $(OBJ_START) $(OBJ_T_H) $(LDLIBS) -o $@
$(TARGET_L).elf: $(OBJ_L) $(OBJ_SRC) $(OBJ_START) $(OBJ_T_L) 
	$(LD) $(LDFLAGS_L) $(OBJ_L) $(OBJ_SRC) $(OBJ_START) $(OBJ_T_L) $(LDLIBS) -o $@

$(TARGET_H).srec: $(TARGET_H).elf
	$(OBJCOPY) -O srec $< $@
$(TARGET_L).srec: $(TARGET_L).elf
	$(OBJCOPY) -O srec $< $@

$(TARGET_H).binary: $(TARGET_H).elf
	$(OBJCOPY) -O binary $< $@
$(TARGET_L).binary: $(TARGET_L).elf
	$(OBJCOPY) -O binary $< $@

size_h:
	$(SIZE) $(TARGET_H).elf
size_l:
	$(SIZE) $(TARGET_L).elf

flash_led: all
	openocd -f openocd.cfg -c "program $(TARGET_L).elf verify reset exit"

flash_hello: all
	openocd -f openocd.cfg -c "program $(TARGET_H).elf verify reset exit"