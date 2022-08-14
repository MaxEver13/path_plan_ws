;; Auto-generated. Do not edit!


(when (boundp 'hybrid_astar::GlobalPath)
  (if (not (find-package "HYBRID_ASTAR"))
    (make-package "HYBRID_ASTAR"))
  (shadow 'GlobalPath (find-package "HYBRID_ASTAR")))
(unless (find-package "HYBRID_ASTAR::GLOBALPATH")
  (make-package "HYBRID_ASTAR::GLOBALPATH"))
(unless (find-package "HYBRID_ASTAR::GLOBALPATHREQUEST")
  (make-package "HYBRID_ASTAR::GLOBALPATHREQUEST"))
(unless (find-package "HYBRID_ASTAR::GLOBALPATHRESPONSE")
  (make-package "HYBRID_ASTAR::GLOBALPATHRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass hybrid_astar::GlobalPathRequest
  :super ros::object
  :slots ())

(defmethod hybrid_astar::GlobalPathRequest
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

(defclass hybrid_astar::GlobalPathResponse
  :super ros::object
  :slots (_plan ))

(defmethod hybrid_astar::GlobalPathResponse
  (:init
   (&key
    ((:plan __plan) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _plan __plan)
   self)
  (:plan
   (&rest __plan)
   (if (keywordp (car __plan))
       (send* _plan __plan)
     (progn
       (if __plan (setq _plan (car __plan)))
       _plan)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _plan
    (send _plan :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _plan
       (send _plan :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _plan
     (send _plan :deserialize buf ptr-) (incf ptr- (send _plan :serialization-length))
   ;;
   self)
  )

(defclass hybrid_astar::GlobalPath
  :super ros::object
  :slots ())

(setf (get hybrid_astar::GlobalPath :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get hybrid_astar::GlobalPath :datatype-) "hybrid_astar/GlobalPath")
(setf (get hybrid_astar::GlobalPath :request) hybrid_astar::GlobalPathRequest)
(setf (get hybrid_astar::GlobalPath :response) hybrid_astar::GlobalPathResponse)

(defmethod hybrid_astar::GlobalPathRequest
  (:response () (instance hybrid_astar::GlobalPathResponse :init)))

(setf (get hybrid_astar::GlobalPathRequest :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get hybrid_astar::GlobalPathRequest :datatype-) "hybrid_astar/GlobalPathRequest")
(setf (get hybrid_astar::GlobalPathRequest :definition-)
      "
---
nav_msgs/Path plan

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get hybrid_astar::GlobalPathResponse :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get hybrid_astar::GlobalPathResponse :datatype-) "hybrid_astar/GlobalPathResponse")
(setf (get hybrid_astar::GlobalPathResponse :definition-)
      "
---
nav_msgs/Path plan

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :hybrid_astar/GlobalPath "0002bc113c0259d71f6cf8cbc9430e18")


