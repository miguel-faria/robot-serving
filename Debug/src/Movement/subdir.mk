################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Movement/Movement_Manager.cpp \
../src/Movement/manager.cpp 

OBJS += \
./src/Movement/Movement_Manager.o \
./src/Movement/manager.o 

CPP_DEPS += \
./src/Movement/Movement_Manager.d \
./src/Movement/manager.d 


# Each subdirectory must supply rules for building sources it contributes
src/Movement/%.o: ../src/Movement/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


