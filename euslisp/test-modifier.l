#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :test-util "package://jsk_demo_common/euslisp/test-util.l")

(defparameter *current-goal-id* nil)
(defparameter *current-avs* nil)
(defparameter *pub-msg* nil)


(defun feedback-cb (msg)
  (let* ((status (send msg :status))
         (goal-id (send status :goal_id))
         (feedback (send msg :feedback)))
    (when (eq (send status :status) actionlib_msgs::GoalStatus::*preempted*)
      (nreverse *current-avs*)
      (play-from-start)
      )
    (when (not (string= (send goal-id :id) *current-goal-id*))
      (setq *current-goal-id* (send goal-id :id))
      (setq *current-avs* nil))
    (push
     (list (apply-trajectory_point (send feedback :joint_names)
                                   (send feedback :actual)
                                   *pr2*)
           (send (send feedback :actual) :time_from_start))
     *current-avs*)))

(defun play-from-start ()
  (speak-jp "再生します")
  (

(defun main ()
  (ros::roseus "test_modifier")
  (ros::rate 100)
  (ros::load-ros-manifest "jsk_demo_common")
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/feedback" control_msgs::FollowJointTrajectoryActionFeedback #'feedback-cb)
  (ros::advertise "/modify_avs" jsk_demo_common::StampedAngleVector)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

