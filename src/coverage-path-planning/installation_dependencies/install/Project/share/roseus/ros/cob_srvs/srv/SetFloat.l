;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetFloat)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetFloat (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETFLOAT")
  (make-package "COB_SRVS::SETFLOAT"))
(unless (find-package "COB_SRVS::SETFLOATREQUEST")
  (make-package "COB_SRVS::SETFLOATREQUEST"))
(unless (find-package "COB_SRVS::SETFLOATRESPONSE")
  (make-package "COB_SRVS::SETFLOATRESPONSE"))

(in-package "ROS")





(defclass cob_srvs::SetFloatRequest
  :super ros::object
  :slots (_data ))

(defmethod cob_srvs::SetFloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cob_srvs::SetFloatResponse
  :super ros::object
  :slots (_success _message ))

(defmethod cob_srvs::SetFloatResponse
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

(defclass cob_srvs::SetFloat
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetFloat :md5sum-) "96fbe2a49585a736ad8b4a31afbaf099")
(setf (get cob_srvs::SetFloat :datatype-) "cob_srvs/SetFloat")
(setf (get cob_srvs::SetFloat :request) cob_srvs::SetFloatRequest)
(setf (get cob_srvs::SetFloat :response) cob_srvs::SetFloatResponse)

(defmethod cob_srvs::SetFloatRequest
  (:response () (instance cob_srvs::SetFloatResponse :init)))

(setf (get cob_srvs::SetFloatRequest :md5sum-) "96fbe2a49585a736ad8b4a31afbaf099")
(setf (get cob_srvs::SetFloatRequest :datatype-) "cob_srvs/SetFloatRequest")
(setf (get cob_srvs::SetFloatRequest :definition-)
      "float32 data
---
bool success
string message

")

(setf (get cob_srvs::SetFloatResponse :md5sum-) "96fbe2a49585a736ad8b4a31afbaf099")
(setf (get cob_srvs::SetFloatResponse :datatype-) "cob_srvs/SetFloatResponse")
(setf (get cob_srvs::SetFloatResponse :definition-)
      "float32 data
---
bool success
string message

")



(provide :cob_srvs/SetFloat "96fbe2a49585a736ad8b4a31afbaf099")


