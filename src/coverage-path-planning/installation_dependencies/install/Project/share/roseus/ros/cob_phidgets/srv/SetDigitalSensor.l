;; Auto-generated. Do not edit!


(when (boundp 'cob_phidgets::SetDigitalSensor)
  (if (not (find-package "COB_PHIDGETS"))
    (make-package "COB_PHIDGETS"))
  (shadow 'SetDigitalSensor (find-package "COB_PHIDGETS")))
(unless (find-package "COB_PHIDGETS::SETDIGITALSENSOR")
  (make-package "COB_PHIDGETS::SETDIGITALSENSOR"))
(unless (find-package "COB_PHIDGETS::SETDIGITALSENSORREQUEST")
  (make-package "COB_PHIDGETS::SETDIGITALSENSORREQUEST"))
(unless (find-package "COB_PHIDGETS::SETDIGITALSENSORRESPONSE")
  (make-package "COB_PHIDGETS::SETDIGITALSENSORRESPONSE"))

(in-package "ROS")





(defclass cob_phidgets::SetDigitalSensorRequest
  :super ros::object
  :slots (_uri _state ))

(defmethod cob_phidgets::SetDigitalSensorRequest
  (:init
   (&key
    ((:uri __uri) "")
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _uri (string __uri))
   (setq _state (round __state))
   self)
  (:uri
   (&optional __uri)
   (if __uri (setq _uri __uri)) _uri)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; string _uri
    4 (length _uri)
    ;; int8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _uri
       (write-long (length _uri) s) (princ _uri s)
     ;; int8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _uri
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uri (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _state 127) (setq _state (- _state 256)))
   ;;
   self)
  )

(defclass cob_phidgets::SetDigitalSensorResponse
  :super ros::object
  :slots (_uri _state ))

(defmethod cob_phidgets::SetDigitalSensorResponse
  (:init
   (&key
    ((:uri __uri) "")
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _uri (string __uri))
   (setq _state (round __state))
   self)
  (:uri
   (&optional __uri)
   (if __uri (setq _uri __uri)) _uri)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; string _uri
    4 (length _uri)
    ;; int8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _uri
       (write-long (length _uri) s) (princ _uri s)
     ;; int8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _uri
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uri (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _state 127) (setq _state (- _state 256)))
   ;;
   self)
  )

(defclass cob_phidgets::SetDigitalSensor
  :super ros::object
  :slots ())

(setf (get cob_phidgets::SetDigitalSensor :md5sum-) "1ddd624f449abe5e039f3478b75cb4d7")
(setf (get cob_phidgets::SetDigitalSensor :datatype-) "cob_phidgets/SetDigitalSensor")
(setf (get cob_phidgets::SetDigitalSensor :request) cob_phidgets::SetDigitalSensorRequest)
(setf (get cob_phidgets::SetDigitalSensor :response) cob_phidgets::SetDigitalSensorResponse)

(defmethod cob_phidgets::SetDigitalSensorRequest
  (:response () (instance cob_phidgets::SetDigitalSensorResponse :init)))

(setf (get cob_phidgets::SetDigitalSensorRequest :md5sum-) "1ddd624f449abe5e039f3478b75cb4d7")
(setf (get cob_phidgets::SetDigitalSensorRequest :datatype-) "cob_phidgets/SetDigitalSensorRequest")
(setf (get cob_phidgets::SetDigitalSensorRequest :definition-)
      "string uri
int8 state
---
string uri
int8 state
")

(setf (get cob_phidgets::SetDigitalSensorResponse :md5sum-) "1ddd624f449abe5e039f3478b75cb4d7")
(setf (get cob_phidgets::SetDigitalSensorResponse :datatype-) "cob_phidgets/SetDigitalSensorResponse")
(setf (get cob_phidgets::SetDigitalSensorResponse :definition-)
      "string uri
int8 state
---
string uri
int8 state
")



(provide :cob_phidgets/SetDigitalSensor "1ddd624f449abe5e039f3478b75cb4d7")


