################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Vision/Color\ Segmentation/Vision_Processor.cpp \
../src/Vision/Color\ Segmentation/vision_processing.cpp 

OBJS += \
./src/Vision/Color\ Segmentation/Vision_Processor.o \
./src/Vision/Color\ Segmentation/vision_processing.o 

CPP_DEPS += \
./src/Vision/Color\ Segmentation/Vision_Processor.d \
./src/Vision/Color\ Segmentation/vision_processing.d 


# Each subdirectory must supply rules for building sources it contributes
src/Vision/Color\ Segmentation/Vision_Processor.o: ../src/Vision/Color\ Segmentation/Vision_Processor.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"src/Vision/Color Segmentation/Vision_Processor.d" -MT"src/Vision/Color\ Segmentation/Vision_Processor.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/Vision/Color\ Segmentation/vision_processing.o: ../src/Vision/Color\ Segmentation/vision_processing.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/indigo/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"src/Vision/Color Segmentation/vision_processing.d" -MT"src/Vision/Color\ Segmentation/vision_processing.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


