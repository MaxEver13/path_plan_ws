;; Auto-generated. Do not edit!


(when (boundp 'hybrid_astar::MonteCarloSim)
  (if (not (find-package "HYBRID_ASTAR"))
    (make-package "HYBRID_ASTAR"))
  (shadow 'MonteCarloSim (find-package "HYBRID_ASTAR")))
(unless (find-package "HYBRID_ASTAR::MONTECARLOSIM")
  (make-package "HYBRID_ASTAR::MONTECARLOSIM"))
(unless (find-package "HYBRID_ASTAR::MONTECARLOSIMREQUEST")
  (make-package "HYBRID_ASTAR::MONTECARLOSIMREQUEST"))
(unless (find-package "HYBRID_ASTAR::MONTECARLOSIMRESPONSE")
  (make-package "HYBRID_ASTAR::MONTECARLOSIMRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass hybrid_astar::MonteCarloSimRequest
  :super ros::object
  :slots (_sx _sy _syaw _syaw_t _gx _gy _gyaw ))

(defmethod hybrid_astar::MonteCarloSimRequest
  (:init
   (&key
    ((:sx __sx) 0.0)
    ((:sy __sy) 0.0)
    ((:syaw __syaw) 0.0)
    ((:syaw_t __syaw_t) 0.0)
    ((:gx __gx) 0.0)
    ((:gy __gy) 0.0)
    ((:gyaw __gyaw) 0.0)
    )
   (send-super :init)
   (setq _sx (float __sx))
   (setq _sy (float __sy))
   (setq _syaw (float __syaw))
   (setq _syaw_t (float __syaw_t))
   (setq _gx (float __gx))
   (setq _gy (float __gy))
   (setq _gyaw (float __gyaw))
   self)
  (:sx
   (&optional __sx)
   (if __sx (setq _sx __sx)) _sx)
  (:sy
   (&optional __sy)
   (if __sy (setq _sy __sy)) _sy)
  (:syaw
   (&optional __syaw)
   (if __syaw (setq _syaw __syaw)) _syaw)
  (:syaw_t
   (&optional __syaw_t)
   (if __syaw_t (setq _syaw_t __syaw_t)) _syaw_t)
  (:gx
   (&optional __gx)
   (if __gx (setq _gx __gx)) _gx)
  (:gy
   (&optional __gy)
   (if __gy (setq _gy __gy)) _gy)
  (:gyaw
   (&optional __gyaw)
   (if __gyaw (setq _gyaw __gyaw)) _gyaw)
  (:serialization-length
   ()
   (+
    ;; float32 _sx
    4
    ;; float32 _sy
    4
    ;; float32 _syaw
    4
    ;; float32 _syaw_t
    4
    ;; float32 _gx
    4
    ;; float32 _gy
    4
    ;; float32 _gyaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _sx
       (sys::poke _sx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sy
       (sys::poke _sy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _syaw
       (sys::poke _syaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _syaw_t
       (sys::poke _syaw_t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gx
       (sys::poke _gx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gy
       (sys::poke _gy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyaw
       (sys::poke _gyaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _sx
     (setq _sx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sy
     (setq _sy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _syaw
     (setq _syaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _syaw_t
     (setq _syaw_t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gx
     (setq _gx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gy
     (setq _gy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyaw
     (setq _gyaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass hybrid_astar::MonteCarloSimResponse
  :super ros::object
  :slots (_valid_start _valid_goal _solution_found _path _iterations _nodes _execution_time ))

(defmethod hybrid_astar::MonteCarloSimResponse
  (:init
   (&key
    ((:valid_start __valid_start) nil)
    ((:valid_goal __valid_goal) nil)
    ((:solution_found __solution_found) nil)
    ((:path __path) (instance nav_msgs::Path :init))
    ((:iterations __iterations) 0)
    ((:nodes __nodes) 0)
    ((:execution_time __execution_time) 0)
    )
   (send-super :init)
   (setq _valid_start __valid_start)
   (setq _valid_goal __valid_goal)
   (setq _solution_found __solution_found)
   (setq _path __path)
   (setq _iterations (round __iterations))
   (setq _nodes (round __nodes))
   (setq _execution_time (round __execution_time))
   self)
  (:valid_start
   (&optional __valid_start)
   (if __valid_start (setq _valid_start __valid_start)) _valid_start)
  (:valid_goal
   (&optional __valid_goal)
   (if __valid_goal (setq _valid_goal __valid_goal)) _valid_goal)
  (:solution_found
   (&optional __solution_found)
   (if __solution_found (setq _solution_found __solution_found)) _solution_found)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:iterations
   (&optional __iterations)
   (if __iterations (setq _iterations __iterations)) _iterations)
  (:nodes
   (&optional __nodes)
   (if __nodes (setq _nodes __nodes)) _nodes)
  (:execution_time
   (&optional __execution_time)
   (if __execution_time (setq _execution_time __execution_time)) _execution_time)
  (:serialization-length
   ()
   (+
    ;; bool _valid_start
    1
    ;; bool _valid_goal
    1
    ;; bool _solution_found
    1
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ;; uint64 _iterations
    8
    ;; uint64 _nodes
    8
    ;; uint64 _execution_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _valid_start
       (if _valid_start (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid_goal
       (if _valid_goal (write-byte -1 s) (write-byte 0 s))
     ;; bool _solution_found
       (if _solution_found (write-byte -1 s) (write-byte 0 s))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;; uint64 _iterations
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _iterations (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _iterations) (= (length (_iterations . bv)) 2)) ;; bignum
              (write-long (ash (elt (_iterations . bv) 0) 0) s)
              (write-long (ash (elt (_iterations . bv) 1) -1) s))
             ((and (class _iterations) (= (length (_iterations . bv)) 1)) ;; big1
              (write-long (elt (_iterations . bv) 0) s)
              (write-long (if (>= _iterations 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _iterations s)(write-long (if (>= _iterations 0) 0 #xffffffff) s)))
     ;; uint64 _nodes
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _nodes (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _nodes) (= (length (_nodes . bv)) 2)) ;; bignum
              (write-long (ash (elt (_nodes . bv) 0) 0) s)
              (write-long (ash (elt (_nodes . bv) 1) -1) s))
             ((and (class _nodes) (= (length (_nodes . bv)) 1)) ;; big1
              (write-long (elt (_nodes . bv) 0) s)
              (write-long (if (>= _nodes 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _nodes s)(write-long (if (>= _nodes 0) 0 #xffffffff) s)))
     ;; uint64 _execution_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _execution_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _execution_time) (= (length (_execution_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_execution_time . bv) 0) 0) s)
              (write-long (ash (elt (_execution_time . bv) 1) -1) s))
             ((and (class _execution_time) (= (length (_execution_time . bv)) 1)) ;; big1
              (write-long (elt (_execution_time . bv) 0) s)
              (write-long (if (>= _execution_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _execution_time s)(write-long (if (>= _execution_time 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _valid_start
     (setq _valid_start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid_goal
     (setq _valid_goal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _solution_found
     (setq _solution_found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; uint64 _iterations
     
#+(or :alpha :irix6 :x86_64)
      (setf _iterations (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _iterations (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _nodes
     
#+(or :alpha :irix6 :x86_64)
      (setf _nodes (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _nodes (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _execution_time
     
#+(or :alpha :irix6 :x86_64)
      (setf _execution_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _execution_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass hybrid_astar::MonteCarloSim
  :super ros::object
  :slots ())

(setf (get hybrid_astar::MonteCarloSim :md5sum-) "a66cae901b07bf73b804e8e47bac5076")
(setf (get hybrid_astar::MonteCarloSim :datatype-) "hybrid_astar/MonteCarloSim")
(setf (get hybrid_astar::MonteCarloSim :request) hybrid_astar::MonteCarloSimRequest)
(setf (get hybrid_astar::MonteCarloSim :response) hybrid_astar::MonteCarloSimResponse)

(defmethod hybrid_astar::MonteCarloSimRequest
  (:response () (instance hybrid_astar::MonteCarloSimResponse :init)))

(setf (get hybrid_astar::MonteCarloSimRequest :md5sum-) "a66cae901b07bf73b804e8e47bac5076")
(setf (get hybrid_astar::MonteCarloSimRequest :datatype-) "hybrid_astar/MonteCarloSimRequest")
(setf (get hybrid_astar::MonteCarloSimRequest :definition-)
      "float32 sx
float32 sy
float32 syaw
float32 syaw_t
float32 gx
float32 gy
float32 gyaw
---
bool valid_start
bool valid_goal
bool solution_found
nav_msgs/Path path
uint64 iterations
uint64 nodes
uint64 execution_time

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

(setf (get hybrid_astar::MonteCarloSimResponse :md5sum-) "a66cae901b07bf73b804e8e47bac5076")
(setf (get hybrid_astar::MonteCarloSimResponse :datatype-) "hybrid_astar/MonteCarloSimResponse")
(setf (get hybrid_astar::MonteCarloSimResponse :definition-)
      "float32 sx
float32 sy
float32 syaw
float32 syaw_t
float32 gx
float32 gy
float32 gyaw
---
bool valid_start
bool valid_goal
bool solution_found
nav_msgs/Path path
uint64 iterations
uint64 nodes
uint64 execution_time

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



(provide :hybrid_astar/MonteCarloSim "a66cae901b07bf73b804e8e47bac5076")


