
(cl:in-package :asdf)

(defsystem "robot_serving-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Cups" :depends-on ("_package_Cups"))
    (:file "_package_Cups" :depends-on ("_package"))
    (:file "PMPTraj" :depends-on ("_package_PMPTraj"))
    (:file "_package_PMPTraj" :depends-on ("_package"))
    (:file "PMPPoint" :depends-on ("_package_PMPPoint"))
    (:file "_package_PMPPoint" :depends-on ("_package"))
  ))