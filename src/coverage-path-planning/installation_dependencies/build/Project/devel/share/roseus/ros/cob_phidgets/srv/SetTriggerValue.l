;; Auto-generated. Do not edit!


(when (boundp 'cob_phidgets::SetTriggerValue)
  (if (not (find-package "COB_PHIDGETS"))
    (make-package "COB_PHIDGETS"))
  (shadow 'SetTriggerValue (find-package "COB_PHIDGETS")))
(unless (find-package "COB_PHIDGETS::SETTRIGGERVALUE")
  (make-package "COB_PHIDGETS::SETTRIGGERVALUE"))
(unless (find-package "COB_PHIDGETS::SETTRIGGERVALUEREQUEST")
  (make-package "COB_PHIDGETS::SETTRIGGERVALUEREQUEST"))
(unless (find-package "COB_PHIDGETS::SETTRIGGERVALUERESPONSE")
  (make-package "COB_PHIDGETS::SETTRIGGERVALUERESPONSE"))

(in-package "ROS")





(defclass cob_phidgets::SetTriggerValueRequest
  :super ros::object
  :slots (_index _trigger_value ))

(defmethod cob_phidgets::SetTriggerValueRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:trigger_value __trigger_value) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _trigger_value (round __trigger_value))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:trigger_value
   (&optional __trigger_value)
   (if __trigger_value (setq _trigger_value __trigger_value)) _trigger_value)
  (:serialization-length
   ()
   (+
    ;; int8 _index
    1
    ;; uint16 _trigger_value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _index
       (write-byte _index s)
     ;; uint16 _trigger_value
       (write-word _trigger_value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;; uint16 _trigger_value
     (setq _trigger_value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass cob_phidgets::SetTriggerValueResponse
  :super ros::object
  :slots ())

(defmethod cob_phidgets::SetTriggerValueResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cob_phidgets::SetTriggerValue
  :super ros::object
  :slots ())

(setf (get cob_phidgets::SetTriggerValue :md5sum-) "f5eabd2487a9b62214c60cbc4be257ee")
(setf (get cob_phidgets::SetTriggerValue :datatype-) "cob_phidgets/SetTriggerValue")
(setf (get cob_phidgets::SetTriggerValue :request) cob_phidgets::SetTriggerValueRequest)
(setf (get cob_phidgets::SetTriggerValue :response) cob_phidgets::SetTriggerValueResponse)

(defmethod cob_phidgets::SetTriggerValueRequest
  (:response () (instance cob_phidgets::SetTriggerValueResponse :init)))

(setf (get cob_phidgets::SetTriggerValueRequest :md5sum-) "f5eabd2487a9b62214c60cbc4be257ee")
(setf (get cob_phidgets::SetTriggerValueRequest :datatype-) "cob_phidgets/SetTriggerValueRequest")
(setf (get cob_phidgets::SetTriggerValueRequest :definition-)
      "int8 index
uint16 trigger_value
---
")

(setf (get cob_phidgets::SetTriggerValueResponse :md5sum-) "f5eabd2487a9b62214c60cbc4be257ee")
(setf (get cob_phidgets::SetTriggerValueResponse :datatype-) "cob_phidgets/SetTriggerValueResponse")
(setf (get cob_phidgets::SetTriggerValueResponse :definition-)
      "int8 index
uint16 trigger_value
---
")



(provide :cob_phidgets/SetTriggerValue "f5eabd2487a9b62214c60cbc4be257ee")


