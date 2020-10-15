;; Auto-generated. Do not edit!


(when (boundp 'MainRobot::motor_serial)
  (if (not (find-package "MAINROBOT"))
    (make-package "MAINROBOT"))
  (shadow 'motor_serial (find-package "MAINROBOT")))
(unless (find-package "MAINROBOT::MOTOR_SERIAL")
  (make-package "MAINROBOT::MOTOR_SERIAL"))
(unless (find-package "MAINROBOT::MOTOR_SERIALREQUEST")
  (make-package "MAINROBOT::MOTOR_SERIALREQUEST"))
(unless (find-package "MAINROBOT::MOTOR_SERIALRESPONSE")
  (make-package "MAINROBOT::MOTOR_SERIALRESPONSE"))

(in-package "ROS")





(defclass MainRobot::motor_serialRequest
  :super ros::object
  :slots (_id _cmd _data ))

(defmethod MainRobot::motor_serialRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:cmd __cmd) 0)
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _cmd (round __cmd))
   (setq _data (round __data))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; uint8 _cmd
    1
    ;; int16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8 _cmd
       (write-byte _cmd s)
     ;; int16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass MainRobot::motor_serialResponse
  :super ros::object
  :slots (_data ))

(defmethod MainRobot::motor_serialResponse
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass MainRobot::motor_serial
  :super ros::object
  :slots ())

(setf (get MainRobot::motor_serial :md5sum-) "145fd4bef55a3bc00166d50e6ebdc608")
(setf (get MainRobot::motor_serial :datatype-) "MainRobot/motor_serial")
(setf (get MainRobot::motor_serial :request) MainRobot::motor_serialRequest)
(setf (get MainRobot::motor_serial :response) MainRobot::motor_serialResponse)

(defmethod MainRobot::motor_serialRequest
  (:response () (instance MainRobot::motor_serialResponse :init)))

(setf (get MainRobot::motor_serialRequest :md5sum-) "145fd4bef55a3bc00166d50e6ebdc608")
(setf (get MainRobot::motor_serialRequest :datatype-) "MainRobot/motor_serialRequest")
(setf (get MainRobot::motor_serialRequest :definition-)
      "uint8 id
uint8 cmd
int16 data
---
int16 data
")

(setf (get MainRobot::motor_serialResponse :md5sum-) "145fd4bef55a3bc00166d50e6ebdc608")
(setf (get MainRobot::motor_serialResponse :datatype-) "MainRobot/motor_serialResponse")
(setf (get MainRobot::motor_serialResponse :definition-)
      "uint8 id
uint8 cmd
int16 data
---
int16 data
")



(provide :MainRobot/motor_serial "145fd4bef55a3bc00166d50e6ebdc608")


