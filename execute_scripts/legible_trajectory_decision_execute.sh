#!/usr/bin/env bash
source variable_def.sh

echo "------------------------------------------"
echo "-- Legible Trajectory Movement Decision --"
echo "------------------------------------------"
gnome-terminal -e "python $BaxterConnectionExecutable"
$MovementExecutableSimple $CupPosTopic "legible" $LegibleTrajSrv $StartMovSrv $FeedbackSrv $RestartMovSrv $BaxterArm $DecisionPeriod