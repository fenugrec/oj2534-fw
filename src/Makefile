#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!
#
# sauce :	yagarto toolchain (Michael Fischer);
#		modified version @ http://hertaville.com (Hussam Al-Hertani)
# + modifications for oj2534 (fenugrec)
#
##############################################################################################
# 
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# interesting flags :
#RAMBUILD : run from RAM, assumes remap at 0. Only selects a different linker script
RAMBUILD = 1
#to override : "make RAMBUILD=0 ..."
##############################################################################################
# Start of default section
#
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
GDB = $(TRGT)gdb
HEX  = $(CP) -O ihex 
BIN  = $(CP) -O binary -S

MCFLAGS = -mcpu=cortex-m0 -mthumb -mlittle-endian
DBGFLAGS = -gdwarf-2
#gcc flags for ram build :
OPT = -Os
ifeq ($(RAMBUILD), 1)
	OPT += -ffunction-sections
endif

ASFLAGS = $(MCFLAGS) $(DBGFLAGS) -specs=nano.specs -nostartfiles -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(DBGFLAGS) $(OPT) -fomit-frame-pointer -std=c11 -Wall -Wfloat-equal -Wextra -pedantic -Wstrict-prototypes \
	-specs=nano.specs -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
#LDFLAGS is added to -gcc commandline ! it seems $(TRGT)-ld should never be used, according to some sources
# to test : --gc-sections,--print-gc-sections ("garbage collect"; delete unused sections)
LDFLAGS = $(MCFLAGS) -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--gc-sections $(LIBDIR)

# List all default C defines here, like -D_DEBUG=1
DDEFS = -DSTM32F072 -DUSE_STDPERIPH_DRIVER
# List all default ASM defines here, like -D_DEBUG=1
DADEFS =
	
# List all default directories to look for include files here
DINCDIR =
	
# List the default directory to look for the libraries here
DLIBDIR =
	
# List all default libraries here
DLIBS =
	
#
# End of default section
##############################################################################################
	
##############################################################################################
# Start of user section
#
	
# 
# Define project name and Ram/Flash mode here
PROJECT = oj2534-fw
	
# List all user C define here, like -D_DEBUG=1
UDEFS =
	
# Define ASM defines here
UADEFS =

# Root directories
LIBSDIR    = d:/dcs/el/stm32/STM32F0xx_SPL150/Libraries/
STMSPDDIR    = $(LIBSDIR)STM32F0xx_StdPeriph_Driver/

#inc + lib dirs
CMINCDIR = $(LIBSDIR)CMSIS/Include/
DEVDIR  = $(LIBSDIR)CMSIS/Device/ST/STM32F0xx/
# StdPeriph dirs
STMSPSRCDDIR = $(STMSPDDIR)src/
STMSPINCDDIR = $(STMSPDDIR)inc/

#dir for linker scripts
LKRDIR = 

#List C source files here
SRC  = fifos.c
SRC += iso_tx.c
SRC += iso_rx.c
SRC += pmsg.c
SRC += params.c
SRC += utils.c
SRC += timers.c
SRC += main.c
#SRC += startup_stm32f072.s
#SRC += .srcsystem_stm32f0xx.c
#SRC += startup_stm32f0xx.S
## used parts of the STM-Library
#SRC += $(STMSPSRCDDIR)stm32f0xx_adc.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_cec.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_crc.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_comp.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_dac.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_dbgmcu.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_dma.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_exti.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_flash.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_gpio.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_syscfg.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_i2c.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_iwdg.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_pwr.c
SRC += $(STMSPSRCDDIR)stm32f0xx_rcc.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_rtc.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_spi.c
SRC += $(STMSPSRCDDIR)stm32f0xx_tim.c
SRC += $(STMSPSRCDDIR)stm32f0xx_usart.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_wwdg.c
#SRC += $(STMSPSRCDDIR)stm32f0xx_misc.c

# List ASM source files here
ASRC = startup_stm32f072.s
	
# List all user directories here
UINCDIR = $(CMINCDIR) \
 $(DEVDIR)Include \
 $(STMSPINCDDIR) 
#          .inc      

# List the user directory to look for the libraries here
ULIBDIR =
	
# List all user libraries here
ULIBS =
	
#
# End of user defines
##############################################################################################
#
# Define linker script file here
#
ifeq ($(RAMBUILD), 0)
 LDSCRIPT = $(LKRDIR)lkr_flash.ld
 FULL_PRJ = $(PROJECT)_flash
 DEFS    = $(DDEFS) $(UDEFS) -DRAMBUILD=0
 ADEFS   = $(DADEFS) $(UADEFS) -DRAMBUILD=0
else
 LDSCRIPT = $(LKRDIR)lkr_ram2.ld
 FULL_PRJ = $(PROJECT)_ram
 DEFS    = $(DDEFS) $(UDEFS) -DRAMBUILD=1
 ADEFS   = $(DADEFS) $(UADEFS) -DRAMBUILD=1
endif

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))


OBJS  = $(ASRC:.s=.o) $(SRC:.c=.o)
LIBS    = $(DLIBS) $(ULIBS)


# Generate dependency information
CPFLAGS += -MD -MP -MF .dep$(@F).d
	
#
# makefile rules
#

all: $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex $(FULL_PRJ).bin
ifeq ($(RAMBUILD), 1)
	$(TRGT)size $(FULL_PRJ).elf
else
	$(TRGT)size $(FULL_PRJ).elf
endif

%.o: %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%hex: %elf
	$(HEX) $< $@

%bin: %elf
	$(BIN)  $< $@

.PHONY : clean dbg
clean:
	-rm -f $(OBJS)
	-rm -f  $(FULL_PRJ).elf
	-rm -f  $(FULL_PRJ).map
	-rm -f  $(FULL_PRJ).hex
	-rm -f  $(FULL_PRJ).bin
	-rm -f  $(SRC:.c=.c.bak)
	-rm -f  $(SRC:.c=.lst)
	-rm -f  $(ASRC:.s=.s.bak)
	-rm -f  $(ASRC:.s=.lst)
	-rm -Rf .dep
	-rm -f oocd.log
dbg:
	@$(GDB) $(FULL_PRJ).elf -x gdb_oocd.cmds

#gdbn : network gdb through SSH pipe; must have:
#	localhost:3333 forwarded before
#	"openocd -l locallog.log" already running on remote box
gdbn:
	@$(GDB) $(FULL_PRJ).elf -x gdbn_oocd.cmds
# Starting openocd from within gdb : (prob : chokes on big data?)
# 	target remote | openocd -f openocd.cfg -c "gdb_port pipe; log_output oocd.log"
# Load rambuild image, technique qui chie:
#	monitor load_image atomictest_ram.hex
# technique mieux:
#	load
#	break *ResetHandler
#	jump *ResetHandler


# Include the dependency files, should be the last of the makefile
#
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
#23-July-2012 /dev/null gives an error on Win 7 64-bit : Hussam
-include $(shell mkdir -p .dep) $(wildcard .dep/*)
# *** EOF ***
