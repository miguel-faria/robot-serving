################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CMakeFiles/3.5.0/CompilerIdCXX/CMakeCXXCompilerId.cpp 

OBJS += \
./CMakeFiles/3.5.0/CompilerIdCXX/CMakeCXXCompilerId.o 

CPP_DEPS += \
./CMakeFiles/3.5.0/CompilerIdCXX/CMakeCXXCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
CMakeFiles/3.5.0/CompilerIdCXX/%.o: ../CMakeFiles/3.5.0/CompilerIdCXX/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


