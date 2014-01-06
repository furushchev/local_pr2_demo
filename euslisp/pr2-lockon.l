#!/usr/bin/env roseus

;;(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "visualization_msgs")

(ros::roseus "pr2_lockon")

(defparameter *lock-on-shape* (make-cube 10 10 10))

(if (not (boundp '*tfl*)) (setq *tfl* (instance ros::transform-listener :init)))

(defparameter *prev-y* 0)
(defparameter *prev-z* 0)

(defun joy-cb (msg)
  (let* ((axes (send msg :axes))
         (buttons (send msg :buttons))
         (trigger (elt buttons 0))
         (y (elt axes 0))
         (z (elt axes 1)))
    (if (eq y 0)
        (setq y *prev-y*)
        (setq *prev-y* y))
    (if (eq z 0)
        (setq z *prev-z*)
        (setq *prev-z* z))
    (ros::ros-info "axes: ~A~%" axes)
    (ros::ros-info "buttons: ~A~%" buttons)
    (send *lock-on-shape* :reset-coords)
    (send *lock-on-shape* :translate (float-vector 500 (* 500 y) (* 500 z)))
    (ros::publish "/lock_on_marker"
                  (object->marker-msg
                   *lock-on-shape*
                   (instance std_msgs::header :init
                             :stamp (ros::time-now)
                             :frame_id "/head_mount_kinect_rgb_link")
                   :ns "lock_on_marker"
                   ))
    ))

(defun init ()
  (ros::rate 10)
  (ros::subscribe "/joy" sensor_msgs::Joy #'joy-cb)
  (ros::advertise "/lock_on_marker" visualization_msgs::Marker 15)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(init)
