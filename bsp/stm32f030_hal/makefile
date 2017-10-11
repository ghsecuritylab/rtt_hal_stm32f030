TARGET = rtthread-stm32f0xx

FLASH_TARGET  = rtthread
GDB    = arm-none-eabi-gdb

OOCD		 := openocd
OOCDFLAGS	 := -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg
OOCDFLAGS	 += -f /usr/local/share/openocd/scripts/target/stm32f0x.cfg
#OOCDFLAGS        += -f openocd.cfg

.PHONY:all clean flash debug 

all:scons

clean:scons -c

flash: $(FLASH_TARGET).bin
	@printf "  OPEN_OCD FLASH $<\n"
	#$(OOCD) $(OOCDFLAGS) -c "program $(FLASH_TARGET).bin verify reset exit" 
	st-flash write $(FLASH_TARGET).bin 0x8000000

debug: $(TARGET).axf
	@printf "  GDB DEBUG $<\n"
	$(GDB) -iex 'target extended | $(OOCD) $(OOCDFLAGS) -c "gdb_port pipe"' \
	-iex 'monitor reset halt' -ex 'load'  $(TARGET).axf

