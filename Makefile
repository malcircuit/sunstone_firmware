PROJECT_NAME := sunstone

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

SDK_ROOT = /Users/Matt/nRF51_SDK
TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
SOFTDEVICE_HEX = $(SDK_ROOT)/components/softdevice/s110/hex/s110_softdevice.hex

TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

PROJECT_DIR := C:/Users/Matt/workspace/sunstone_firmware
#source common to all targets
C_SOURCE_FILES += \
$(SDK_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
$(SDK_ROOT)/components/drivers_nrf/ppi/nrf_drv_ppi.c \
$(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_ROOT)/components/libraries/pwm/app_pwm.c \
$(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c\
$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c\
$(SDK_ROOT)/components/libraries/util/app_error.c\
$(SDK_ROOT)/components/drivers_nrf/hal/nrf_delay.c \
$(SDK_ROOT)/components/toolchain/system_nrf51.c \
$(SDK_ROOT)/components/libraries/timer/app_timer.c \
$(SDK_ROOT)/components/libraries/trace/app_trace.c \
$(SDK_ROOT)/components/libraries/util/nrf_assert.c \
$(SDK_ROOT)/components/drivers_nrf/pstorage/pstorage.c \
$(SDK_ROOT)/components/ble/common/ble_advdata.c \
$(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
$(SDK_ROOT)/components/ble/common/ble_conn_params.c \
$(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
$(SDK_ROOT)/components/ble/common/ble_srv_common.c \
$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(PROJECT_DIR)/main.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -I$(PROJECT_DIR)
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote/
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/gcc
INC_PATHS += -I$(SDK_ROOT)/components/device
INC_PATHS += -I$(SDK_ROOT)/components/toolchain
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/timer
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/ppi
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util
INC_PATHS += -I$(SDK_ROOT)/components/libraries/pwm
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s110/headers
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/twi_master
INC_PATHS += -I$(SDK_ROOT)/components/ble/common
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_ROOT)/components/libraries/trace
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_nus
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_ROOT)/components/libraries/timer

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY =$(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY =$(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DBL600
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O0 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -flto -fno-builtin

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
#default target - first one defined
default: clean nrf51822_xxaa_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_xxaa_s110 
    
#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_xxaa_s110
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

export OUTPUT_FILENAME := nrf51822_xxaa_s110

nrf51822_xxaa_s110: OUTPUT_FILENAME := nrf51822_xxaa_s110
nrf51822_xxaa_s110: LINKER_SCRIPT = sunstone.ld
nrf51822_xxaa_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	@echo arm-none-eabi-gcc $(CFLAGS) $(INC_PATHS) -c -o $@ $<
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
	
GDB_PORT_NUMBER := 9992

#FLASH_START_ADDR = $(shell $(OBJDUMP) -h $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out -j .text | grep .text | awk '{print $$4}')
FLASH_START_ADDR = 0

JLINK_OPTS = -device nrf51822_xxaa -if swd -speed 4000
JLINK_GDB_OPTS = -noir
JLINKD_GDB = JLinkGDBServer $(JLINK_GDB_OPTS)

# flashing windows vs. posix
ifeq ($(OS),Windows_NT)
JLINK = JLink.exe $(JLINK_OPTS)
else
JLINK = JLinkExe $(JLINK_OPTS)
endif # flashing windows vs. posix

flash: flash.jlink
	$(JLINK) flash.jlink

flash.jlink:
	printf "loadfile $(PROJECT_DIR)/$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex\nr\ng\nexit\n" > flash.jlink

flash_softdevice: flashsd.jlink
	$(JLINK) flashsd.jlink

flashsd.jlink:
	printf "loadbin $(SOFTDEVICE_HEX) 0\nr\ng\nexit\n" > flashsd.jlink

flash-check:
	printf "loadbin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(FLASH_START_ADDR)\nr\ng\nexit\n"

erase-all: erase-all.jlink
	$(JLINK) erase-all.jlink

# Write to NVMC to enable erase, do erase all, wait for completion. reset
erase-all.jlink:
	printf "w4 4001e504 2\nw4 4001e50c 1\nsleep 100\nr\nexit\n" > erase-all.jlink

run-debug:
	$(JLINKD_GDB) $(JLINK_OPTS) $(JLINK_GDB_OPTS) -port $(GDB_PORT_NUMBER)

.PHONY:  flash-jlink flash.jlink erase-all erase-all.jlink run-debug