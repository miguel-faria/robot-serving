
(cl:in-package :asdf)

(defsystem "robot_serving-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :robot_serving-msg
)
  :components ((:file "_package")
    (:file "RobotMovementCancelTrajectory" :depends-on ("_package_RobotMovementCancelTrajectory"))
    (:file "_package_RobotMovementCancelTrajectory" :depends-on ("_package"))
    (:file "Movement" :depends-on ("_package_Movement"))
    (:file "_package_Movement" :depends-on ("_package"))
    (:file "RobotMovementFeedback" :depends-on ("_package_RobotMovementFeedback"))
    (:file "_package_RobotMovementFeedback" :depends-on ("_package"))
    (:file "RobotMovementSendTrajectory" :depends-on ("_package_RobotMovementSendTrajectory"))
    (:file "_package_RobotMovementSendTrajectory" :depends-on ("_package"))
  ))