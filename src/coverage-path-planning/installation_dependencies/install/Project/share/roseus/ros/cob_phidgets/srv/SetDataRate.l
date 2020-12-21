;; Auto-generated. Do not edit!


(when (boundp 'cob_phidgets::SetDataRate)
  (if (not (find-package "COB_PHIDGETS"))
    (make-package "COB_PHIDGETS"))
  (shadow 'SetDataRate (find-package "COB_PHIDGETS")))
(unless (find-package "COB_PHIDGETS::SETDATARATE")
  (make-package "COB_PHIDGETS::SETDATARATE"))
(unless (find-package "COB_PHIDGETS::SETDATARATEREQUEST")
  (make-package "COB_PHIDGETS::SETDATARATEREQUEST"))
(unless (find-package "COB_PHIDGETS::SETDATARATERESPONSE")
  (make-package "COB_PHIDGETS::SETDATARATERESPONSE"))

(in-package "ROS")





(defclass cob_phidgets::SetDataRateRequest
  :super ros::object
  :slots (_index _data_rate ))

(defmethod cob_phidgets::SetDataRateRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:data_rate __data_rate) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _data_rate (round __data_rate))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:data_rate
   (&optional __data_rate)
   (if __data_rate (setq _data_rate __data_rate)) _data_rate)
  (:serialization-length
   ()
   (+
    ;; int8 _index
    1
    ;; uint16 _data_rate
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _index
       (write-byte _index s)
     ;; uint16 _data_rate
       (write-word _data_rate s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;; uint16 _data_rate
     (setq _data_rate (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass cob_phidgets::SetDataRateResponse
  :super ros::object
  :slots ())

(defmethod cob_phidgets::SetDataRateResponse
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

(defclass cob_phidgets::SetDataRate
  :super ros::object
  :slots ())

(setf (get cob_phidgets::SetDataRate :md5sum-) "2fe4d4dafc6b478c7e506a02543a3197")
(setf (get cob_phidgets::SetDataRate :datatype-) "cob_phidgets/SetDataRate")
(setf (get cob_phidgets::SetDataRate :request) cob_phidgets::SetDataRateRequest)
(setf (get cob_phidgets::SetDataRate :response) cob_phidgets::SetDataRateResponse)

(defmethod cob_phidgets::SetDataRateRequest
  (:response () (instance cob_phidgets::SetDataRateResponse :init)))

(setf (get cob_phidgets::SetDataRateRequest :md5sum-) "2fe4d4dafc6b478c7e506a02543a3197")
(setf (get cob_phidgets::SetDataRateRequest :datatype-) "cob_phidgets/SetDataRateRequest")
(setf (get cob_phidgets::SetDataRateRequest :definition-)
      "int8 index
uint16 data_rate

---
")

(setf (get cob_phidgets::SetDataRateResponse :md5sum-) "2fe4d4dafc6b478c7e506a02543a3197")
(setf (get cob_phidgets::SetDataRateResponse :datatype-) "cob_phidgets/SetDataRateResponse")
(setf (get cob_phidgets::SetDataRateResponse :definition-)
      "int8 index
uint16 data_rate

---
")



(provide :cob_phidgets/SetDataRate "2fe4d4dafc6b478c7e506a02543a3197")


