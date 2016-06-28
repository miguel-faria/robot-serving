#!/usr/bin/env bash
source variable_def.sh

echo "----------------------------------------"
echo "-- Mixed Trajectory Movement Decision --"
echo "----------------------------------------"
gnome-terminal -e "python $BaxterConnectionExecutable"
$MovementExecutableSimple $CupPosTopic $LegibleTrajSrv $PredictableTrajSrv $StartMovSrv $FeedbackSrv $RestartMovSrv $BaxterArm $DecisionPeriod