;; Auto-generated. Do not edit!


(when (boundp 'cob_phidgets::AnalogSensor)
  (if (not (find-package "COB_PHIDGETS"))
    (make-package "COB_PHIDGETS"))
  (shadow 'AnalogSensor (find-package "COB_PHIDGETS")))
(unless (find-package "COB_PHIDGETS::ANALOGSENSOR")
  (make-package "COB_PHIDGETS::ANALOGSENSOR"))

(in-package "ROS")
;;//! \htmlinclude AnalogSensor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_phidgets::AnalogSensor
  :super ros::object
  :slots (_header _uri _value ))

(defmethod cob_phidgets::AnalogSensor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uri __uri) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:value __value) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _uri __uri)
   (setq _value __value)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uri
   (&optional __uri)
   (if __uri (setq _uri __uri)) _uri)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _uri
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uri)) 4
    ;; int16[] _value
    (* 2    (length _value)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _uri
     (write-long (length _uri) s)
     (dolist (elem _uri)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int16[] _value
     (write-long (length _value) s)
     (dotimes (i (length _value))
       (write-word (elt _value i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _uri
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _uri (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _uri i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int16[] _value
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _value (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _value i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get cob_phidgets::AnalogSensor :md5sum-) "b52a946d34a1d251ef4461cdadfa70ae")
(setf (get cob_phidgets::AnalogSensor :datatype-) "cob_phidgets/AnalogSensor")
(setf (get cob_phidgets::AnalogSensor :definition-)
      "Header header
string[] uri
int16[] value

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :cob_phidgets/AnalogSensor "b52a946d34a1d251ef4461cdadfa70ae")


