################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Vision/image_converter.cpp \
../src/Vision/video_receiver.cpp 

OBJS += \
./src/Vision/image_converter.o \
./src/Vision/video_receiver.o 

CPP_DEPS += \
./src/Vision/image_converter.d \
./src/Vision/video_receiver.d 


# Each subdirectory must supply rules for building sources it contributes
src/Vision/%.o: ../src/Vision/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


