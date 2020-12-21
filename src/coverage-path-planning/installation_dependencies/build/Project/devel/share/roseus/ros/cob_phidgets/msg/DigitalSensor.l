;; Auto-generated. Do not edit!


(when (boundp 'cob_phidgets::DigitalSensor)
  (if (not (find-package "COB_PHIDGETS"))
    (make-package "COB_PHIDGETS"))
  (shadow 'DigitalSensor (find-package "COB_PHIDGETS")))
(unless (find-package "COB_PHIDGETS::DIGITALSENSOR")
  (make-package "COB_PHIDGETS::DIGITALSENSOR"))

(in-package "ROS")
;;//! \htmlinclude DigitalSensor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_phidgets::DigitalSensor
  :super ros::object
  :slots (_header _uri _state ))

(defmethod cob_phidgets::DigitalSensor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uri __uri) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:state __state) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _uri __uri)
   (setq _state __state)
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
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _uri
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uri)) 4
    ;; int8[] _state
    (* 1    (length _state)) 4
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
     ;; int8[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (write-byte (elt _state i) s)
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
   ;; int8[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _state i) 127) (setf (elt _state i) (- (elt _state i) 256)))
     ))
   ;;
   self)
  )

(setf (get cob_phidgets::DigitalSensor :md5sum-) "a7555c23666cea5795a76c1bdec02d2c")
(setf (get cob_phidgets::DigitalSensor :datatype-) "cob_phidgets/DigitalSensor")
(setf (get cob_phidgets::DigitalSensor :definition-)
      "Header header
string[] uri
int8[] state
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



(provide :cob_phidgets/DigitalSensor "a7555c23666cea5795a76c1bdec02d2c")


