#!/usr/bin/env bash
source variable_def.sh

echo "------------------------------------"
echo "- Launching Vision Processing Node -"
echo "------------------------------------"
$VisionProcessing_Executable $ColorTopic $DepthTopic $CupPosTopic $CalibParams