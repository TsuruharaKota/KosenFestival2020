
(cl:in-package :asdf)

(defsystem "MainRobot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_serial" :depends-on ("_package_motor_serial"))
    (:file "_package_motor_serial" :depends-on ("_package"))
  ))