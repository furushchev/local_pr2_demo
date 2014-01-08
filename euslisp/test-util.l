#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "local_pr2_demo")
(ros::load-ros-manifest "pr2_mechanism_msgs")
;; original package://pr2eus_impedance/euslisp/pr2eus-impedance.l
(defmethod robot-interface
  (:change-controller (stop-controller start-controller)
   (let (req res)
     (setq req (instance pr2_mechanism_msgs::SwitchControllerRequest :init))
     (send req :stop_controllers (list stop-controller))
     (send req :start_controllers (list start-controller))
     (send req :strictness 2)
     (setq res (ros::service-call "/pr2_controller_manager/switch_controller" req))
     (ros::ros-warn "changing controller from ~A to ~A" stop-controller start-controller)
     (unless (send res :ok)
       (ros::ros-warn "failed to change controller")
       )
     ))
  (:start-mannequin-mode (&optional controller)
   (case controller
     (:rarm
      (send self :change-controller "r_arm_controller" "r_arm_controller_loose"))
     (:larm
      (send self :change-controller "l_arm_controller" "l_arm_controller_loose"))
     (:head
      (send self :change-controller "head_traj_controller" "head_traj_controller_loose"))
     (:arms
      (send self :start-mannequin-mode :larm)
      (send self :start-mannequin-mode :rarm))
     (t
      (send self :start-mannequin-mode :arms)
      (send self :start-mannequin-mode :head))))
  (:stop-mannequin-mode (&optional controller)
   (case controller
     (:rarm
      (send self :change-controller "r_arm_controller_loose" "r_arm_controller"))
     (:larm
      (send self :change-controller "l_arm_controller_loose" "l_arm_controller"))
     (:head
      (send self :change-controller "head_traj_controller_loose" "head_traj_controller"))
     (:arms
      (send self :stop-mannequin-mode :larm)
      (send self :stop-mannequin-mode :rarm))
     (t
      (send self :stop-mannequin-mode :arms)
      (send self :stop-mannequin-mode :head))))

  ) ;; defmethod robot-interface

;; original package://pr2eus/robot-interface.l
(defun apply-trajectory_point (names trajpoint robot)
  (let ((cntr 0)
        (ps (send trajpoint :positions))
        )
    (dolist (jn names)
      (let ((j (send robot (intern (string-upcase jn) *keyword-package*))))
        (send j :ros-joint-angle (elt ps cntr))
        (incf cntr)))
    robot))

;; conversion
(defun ros->eus (msg)
  "local_pr2_demo::AngleVectorSequence -> (avs tms)"
  (let ((ros-av-lst (send msg :data))
        avs tms)
    (dolist (ros-av ros-av-lst)
      (push (send ros-av :data) avs)
      (push (send ros-av :dura) tms))
    (list (nreverse avs) (nreverse tms))))

(defun eus->ros (avs-tm-lst &key (id nil))
  "(avs tms) -> local_pr2_demo::AngleVectorSequence"
  (let ((msg (instance local_pr2_demo::AngleVectorSequence :init
                       :id id))
        ros-av-lst)
    (dotimes (i (length (car avs-tm-lst)))
      (push (instance local_pr2_demo::AngleVector :init
                      :data (elt (car avs-tm-lst) i)
                      :dura (elt (cadr avs-tm-lst) i))
            ros-av-lst))
    (send msg :data (nreverse ros-av-lst))
    msg))

(defparameter *angle-vector-flag* "/angle_vector_flag")

(provide :test-util)
