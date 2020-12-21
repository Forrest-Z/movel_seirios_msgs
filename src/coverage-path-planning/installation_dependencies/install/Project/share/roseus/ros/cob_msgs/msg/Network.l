;; Auto-generated. Do not edit!


(when (boundp 'cob_msgs::Network)
  (if (not (find-package "COB_MSGS"))
    (make-package "COB_MSGS"))
  (shadow 'Network (find-package "COB_MSGS")))
(unless (find-package "COB_MSGS::NETWORK")
  (make-package "COB_MSGS::NETWORK"))

(in-package "ROS")
;;//! \htmlinclude Network.msg.html


(defclass cob_msgs::Network
  :super ros::object
  :slots (_macattr _essid _channel _rssi _noise _beacon ))

(defmethod cob_msgs::Network
  (:init
   (&key
    ((:macattr __macattr) "")
    ((:essid __essid) "")
    ((:channel __channel) 0)
    ((:rssi __rssi) 0)
    ((:noise __noise) 0)
    ((:beacon __beacon) 0)
    )
   (send-super :init)
   (setq _macattr (string __macattr))
   (setq _essid (string __essid))
   (setq _channel (round __channel))
   (setq _rssi (round __rssi))
   (setq _noise (round __noise))
   (setq _beacon (round __beacon))
   self)
  (:macattr
   (&optional __macattr)
   (if __macattr (setq _macattr __macattr)) _macattr)
  (:essid
   (&optional __essid)
   (if __essid (setq _essid __essid)) _essid)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:rssi
   (&optional __rssi)
   (if __rssi (setq _rssi __rssi)) _rssi)
  (:noise
   (&optional __noise)
   (if __noise (setq _noise __noise)) _noise)
  (:beacon
   (&optional __beacon)
   (if __beacon (setq _beacon __beacon)) _beacon)
  (:serialization-length
   ()
   (+
    ;; string _macattr
    4 (length _macattr)
    ;; string _essid
    4 (length _essid)
    ;; int32 _channel
    4
    ;; int32 _rssi
    4
    ;; int32 _noise
    4
    ;; int32 _beacon
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _macattr
       (write-long (length _macattr) s) (princ _macattr s)
     ;; string _essid
       (write-long (length _essid) s) (princ _essid s)
     ;; int32 _channel
       (write-long _channel s)
     ;; int32 _rssi
       (write-long _rssi s)
     ;; int32 _noise
       (write-long _noise s)
     ;; int32 _beacon
       (write-long _beacon s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _macattr
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _macattr (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _essid
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _essid (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _channel
     (setq _channel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rssi
     (setq _rssi (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _noise
     (setq _noise (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _beacon
     (setq _beacon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cob_msgs::Network :md5sum-) "b0854419660dc197dd94305843bee07f")
(setf (get cob_msgs::Network :datatype-) "cob_msgs/Network")
(setf (get cob_msgs::Network :definition-)
      "string macattr
string essid
int32 channel
int32 rssi
int32 noise
int32 beacon

")



(provide :cob_msgs/Network "b0854419660dc197dd94305843bee07f")


