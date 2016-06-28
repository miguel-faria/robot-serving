################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/Vision/Color\ Segmentation/subdir.mk
-include src/Vision/subdir.mk
-include src/Movement/subdir.mk
-include CMakeFiles/vision_receiver.dir/src/Vision/subdir.mk
-include CMakeFiles/vision_processing_image_converter.dir/src/Vision/subdir.mk
-include CMakeFiles/vision_processing_color_segmentation.dir/src/Vision/Color_Segmentation/subdir.mk
-include CMakeFiles/movement_manager.dir/src/Movement/subdir.mk
-include CMakeFiles/3.5.0/CompilerIdCXX/subdir.mk
-include CMakeFiles/3.5.0/CompilerIdC/subdir.mk
-include CMakeFiles/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Thesis_Robot_Serving

# Tool invocations
Thesis_Robot_Serving: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++  -o "Thesis_Robot_Serving" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) Thesis_Robot_Serving
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
