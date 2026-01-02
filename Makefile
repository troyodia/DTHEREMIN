.PHONY: all clean flash cppcheck format
#check arguments
ifneq ($(TEST),) 
ifeq ($(findstring test_, $(TEST)),)
$(error "TEST=$(TEST) is not a vaild test name, test must start with test_")
else
TARGET_NAME=$(TEST)
endif 
else
TARGET_NAME=THEREMIN
endif
######################################
# target
######################################
# TARGET = SUMO_ROBOT
TARGET = $(TARGET_NAME)

#defines
TEST_DEFINE=$(addprefix -DTEST=, $(TEST))
PRINTF_CONFIG_DEFINE=$(addprefix -D, $(PRINTF_INCLUDE_CONFIG_H))
DEFINES= \
	$(TEST_DEFINE) \
	$(PRINTF_CONFIG_DEFINE) \
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj/$(TARGET)
BIN_DIR = $(BUILD_DIR)/bin/$(TARGET)
CPPCHECK = cppcheck 
FORMAT = clang-format
CPPCHECK_INCLUDES = \
	./Src/app \
	./Src/common \
	./Src/drivers \
	./Src/test \

IGNORE_FILES_FORMAT_CPPCHECK = \
	external/printf/printf.c \
	external/printf/printf.h

SOURCES_FORMAT_CPPCHECK = $(filter-out $(IGNORE_FILES_FORMAT_CPPCHECK), $(CPP_CHECK_C_SOURCES))
HEADERS_FORMAT_CPPCHECK = $(filter-out $(IGNORE_FILES_FORMAT_CPPCHECK), $(HEADERS))

CPPCHECK_FLAGS = \
	--quiet --enable=all --error-exitcode=1 \
	--inline-suppr \
	--suppress=missingIncludeSystem \
	--suppress=missingInclude \
	--suppress=unmatchedSuppression \
	--suppress=unusedFunction \
	$(addprefix -I,$(CPPCHECK_INCLUDES)) 	

######################################
# source
######################################
# C sources

C_SOURCES_WITH_HEADERS = \
	Src/drivers/map.c \
	Src/drivers/dds.c \
	Src/drivers/lp_filter.c \
	Src/drivers/io.c \
	Src/drivers/mcu_init.c \
	Src/drivers/led.c \
	Src/drivers/uart.c \
	Src/drivers/gp2y0a41sk0f.c \
	Src/drivers/pwm.c \
	Src/drivers/adc.c \
	Src/common/ring_buffer.c \
	Src/common/assert_handler.c \
	Src/common/trace.c \
	Src/common/systick.c \
	external/printf/printf.c \

ifndef TEST
C_SOURCES =  \
	Src/main.c \
	$(C_SOURCES_WITH_HEADERS) \
	Src/system_stm32l4xx.c \

CPP_CHECK_C_SOURCES =  \
	Src/main.c \
	$(C_SOURCES_WITH_HEADERS) \

else
#delete test.o before new build to ensure new test is always built
$(shell rm -rf $(OBJ_DIR)/Src/test/test.o)

C_SOURCES =  \
	Src/test/test.c \
	Src/common/errno_stub.c \
	$(C_SOURCES_WITH_HEADERS) \
	Src/system_stm32l4xx.c \

CPP_CHECK_C_SOURCES =  \
	Src/test/test.c \
	$(C_SOURCES_WITH_HEADERS) \

endif

HEADERS = \
	$(C_SOURCES_WITH_HEADERS:.c=.h) \
	Src/common/defines.h \

# ASM sources
ASM_SOURCES =  \
	Startup/startup_stm32l476rgtx.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
GCC_DIR = $(GCC_PATH)
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_DIR variable (> make GCC_DIR=xxx)
# The gcc compiler bin path can be either defined in make command via GCC_DIR variable (> make GCC_DIR=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_DIR
CC = $(GCC_DIR)/$(PREFIX)gcc
AS = $(GCC_DIR)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_DIR)/$(PREFIX)objcopy
SZ = $(GCC_DIR)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
# HEX = $(CP) -O ihex
# BIN = $(CP) -O binary -S
READELF=$(GCC_DIR)/$(PREFIX)readelf 
ADDR2LINE=$(GCC_DIR)/$(PREFIX)addr2line 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
  
# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32L476xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
	Drivers/CMSIS/Device/ST/STM32L4xx/Include \
	Drivers/CMSIS/Include \
	Src/app \
	Src/common \
	Src/drivers \
	Src/test \
	external/printf \



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(DEFINES) $(OPT) -g -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(addprefix -I,$(C_INCLUDES)) $(OPT) $(DEFINES)  -g -gdwarf-2 -fshort-enums -Wall -Wextra -Werror -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L476RGTX_FLASH.ld

# libraries
LIBS = -lm -lc -lnosys 
LIBDIR = 
LDFLAGS += $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS)  -g -Wl,--gc-sections

# default action: build all
all: $(BIN_DIR)/$(TARGET).elf 


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(OBJ_DIR)/, $(C_SOURCES:.c=.o))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(OBJ_DIR)/,$(ASM_SOURCES:.s=.o))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(OBJ_DIR)/,$(ASMM_SOURCES:.S=.o))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))
	

$(OBJ_DIR)/%.o: %.c Makefile 
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o: %.S Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(CFLAGS) $< -o $@

$(BIN_DIR)/$(TARGET).elf: $(OBJECTS) $(HEADERS) 
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) $^ -o $@

	
#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
# openocd
#######################################
flash: all
	openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program $(BIN_DIR)/$(TARGET).elf verify reset exit"
#######################################
cppcheck: 
	@$(CPPCHECK) $(CPPCHECK_FLAGS) $(SOURCES_FORMAT_CPPCHECK)
#######################################
format:
	@$(FORMAT) -i $(SOURCES_FORMAT_CPPCHECK) $(HEADERS_FORMAT_CPPCHECK)

size: all
	$(SZ) $(BIN_DIR)/$(TARGET).elf

symbols: all
	$(READELF) -s $(BIN_DIR)/$(TARGET).elf | sort -n -k3

addr2line: all
	$(ADDR2LINE) -e $(BIN_DIR)/$(TARGET).elf $(ADDR)
# *** EOF ***
