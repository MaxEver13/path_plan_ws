;; Auto-generated. Do not edit!


(when (boundp 'hybrid_astar::Test)
  (if (not (find-package "HYBRID_ASTAR"))
    (make-package "HYBRID_ASTAR"))
  (shadow 'Test (find-package "HYBRID_ASTAR")))
(unless (find-package "HYBRID_ASTAR::TEST")
  (make-package "HYBRID_ASTAR::TEST"))

(in-package "ROS")
;;//! \htmlinclude Test.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass hybrid_astar::Test
  :super ros::object
  :slots (_sx _sy _syaw _syaw_t _gx _gy _gyaw _solution_found _iterations _nodes _execution_time _path ))

(defmethod hybrid_astar::Test
  (:init
   (&key
    ((:sx __sx) 0.0)
    ((:sy __sy) 0.0)
    ((:syaw __syaw) 0.0)
    ((:syaw_t __syaw_t) 0.0)
    ((:gx __gx) 0.0)
    ((:gy __gy) 0.0)
    ((:gyaw __gyaw) 0.0)
    ((:solution_found __solution_found) nil)
    ((:iterations __iterations) 0)
    ((:nodes __nodes) 0)
    ((:execution_time __execution_time) 0)
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _sx (float __sx))
   (setq _sy (float __sy))
   (setq _syaw (float __syaw))
   (setq _syaw_t (float __syaw_t))
   (setq _gx (float __gx))
   (setq _gy (float __gy))
   (setq _gyaw (float __gyaw))
   (setq _solution_found __solution_found)
   (setq _iterations (round __iterations))
   (setq _nodes (round __nodes))
   (setq _execution_time (round __execution_time))
   (setq _path __path)
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
  (:solution_found
   (&optional __solution_found)
   (if __solution_found (setq _solution_found __solution_found)) _solution_found)
  (:iterations
   (&optional __iterations)
   (if __iterations (setq _iterations __iterations)) _iterations)
  (:nodes
   (&optional __nodes)
   (if __nodes (setq _nodes __nodes)) _nodes)
  (:execution_time
   (&optional __execution_time)
   (if __execution_time (setq _execution_time __execution_time)) _execution_time)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
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
    ;; bool _solution_found
    1
    ;; uint64 _iterations
    8
    ;; uint64 _nodes
    8
    ;; uint64 _execution_time
    8
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
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
     ;; bool _solution_found
       (if _solution_found (write-byte -1 s) (write-byte 0 s))
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
     ;; nav_msgs/Path _path
       (send _path :serialize s)
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
   ;; bool _solution_found
     (setq _solution_found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
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
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(setf (get hybrid_astar::Test :md5sum-) "9389d4240b0adb885c3654290c1b04b8")
(setf (get hybrid_astar::Test :datatype-) "hybrid_astar/Test")
(setf (get hybrid_astar::Test :definition-)
      "float32 sx
float32 sy
float32 syaw
float32 syaw_t
float32 gx
float32 gy
float32 gyaw
bool solution_found
uint64 iterations
uint64 nodes
uint64 execution_time
nav_msgs/Path path
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



(provide :hybrid_astar/Test "9389d4240b0adb885c3654290c1b04b8")


