
(cl:in-package :asdf)

(defsystem "cob_map_accessibility_analysis-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "CheckPerimeterAccessibility" :depends-on ("_package_CheckPerimeterAccessibility"))
    (:file "_package_CheckPerimeterAccessibility" :depends-on ("_package"))
    (:file "CheckPointAccessibility" :depends-on ("_package_CheckPointAccessibility"))
    (:file "_package_CheckPointAccessibility" :depends-on ("_package"))
  ))