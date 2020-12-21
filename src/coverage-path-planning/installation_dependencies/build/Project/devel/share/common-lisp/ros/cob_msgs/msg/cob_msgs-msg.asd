
(cl:in-package :asdf)

(defsystem "cob_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :diagnostic_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AccessPoint" :depends-on ("_package_AccessPoint"))
    (:file "_package_AccessPoint" :depends-on ("_package"))
    (:file "DashboardState" :depends-on ("_package_DashboardState"))
    (:file "_package_DashboardState" :depends-on ("_package"))
    (:file "EmergencyStopState" :depends-on ("_package_EmergencyStopState"))
    (:file "_package_EmergencyStopState" :depends-on ("_package"))
    (:file "Network" :depends-on ("_package_Network"))
    (:file "_package_Network" :depends-on ("_package"))
    (:file "PowerState" :depends-on ("_package_PowerState"))
    (:file "_package_PowerState" :depends-on ("_package"))
    (:file "SiteSurvey" :depends-on ("_package_SiteSurvey"))
    (:file "_package_SiteSurvey" :depends-on ("_package"))
  ))