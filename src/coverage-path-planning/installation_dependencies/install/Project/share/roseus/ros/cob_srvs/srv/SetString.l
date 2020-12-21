;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetString)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetString (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETSTRING")
  (make-package "COB_SRVS::SETSTRING"))
(unless (find-package "COB_SRVS::SETSTRINGREQUEST")
  (make-package "COB_SRVS::SETSTRINGREQUEST"))
(unless (find-package "COB_SRVS::SETSTRINGRESPONSE")
  (make-package "COB_SRVS::SETSTRINGRESPONSE"))

(in-package "ROS")





(defclass cob_srvs::SetStringRequest
  :super ros::object
  :slots (_data ))

(defmethod cob_srvs::SetStringRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cob_srvs::SetStringResponse
  :super ros::object
  :slots (_success _message ))

(defmethod cob_srvs::SetStringResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cob_srvs::SetString
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetString :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get cob_srvs::SetString :datatype-) "cob_srvs/SetString")
(setf (get cob_srvs::SetString :request) cob_srvs::SetStringRequest)
(setf (get cob_srvs::SetString :response) cob_srvs::SetStringResponse)

(defmethod cob_srvs::SetStringRequest
  (:response () (instance cob_srvs::SetStringResponse :init)))

(setf (get cob_srvs::SetStringRequest :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get cob_srvs::SetStringRequest :datatype-) "cob_srvs/SetStringRequest")
(setf (get cob_srvs::SetStringRequest :definition-)
      "string data
---
bool success
string message

")

(setf (get cob_srvs::SetStringResponse :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get cob_srvs::SetStringResponse :datatype-) "cob_srvs/SetStringResponse")
(setf (get cob_srvs::SetStringResponse :definition-)
      "string data
---
bool success
string message

")



(provide :cob_srvs/SetString "546971982e3fbbd5a41e60fb6432e357")


