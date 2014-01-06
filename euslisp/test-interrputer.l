#!/usr/bin/env roseus
(ros::roseus "test_interrupter")
(ros::load-ros-manifest "local_pr2_demo")

(load "package://jsk_demo_common/euslisp/attention-observation.l")
(require :test-util "test-util.l")

(defparameter *last-goal-id* nil)
(defparameter *last-goal* nil)
(defparameter *last-feedback* nil)

(defun feedback-cb (msg)
  (warn "feedback-cb~%")
  (let* ((status (send msg :status))
         (goal-id (send status :goal_id))
         (feedback (send msg :feedback)))
    (when (and (eq *last-goal-id* (send goal-id :id))
               (eq (send status :status) actionlib_msgs::GoalStatus::*preempted*))
      (setq *last-feedback* feedback))))

(defun goal-cb (msg)
  (warn "goal-cb~%")
  (let* ((goal-id (send msg :goal_id))
         (goal (send msg :goal)))
    (setq *last-goal-id* (send goal-id :id))
    (setq *last-goal* goal)))

(defun wait-torque ()
  (do-until-key
      (ros::spin-once)
    (send *ri* :state)
    (let ((mtrq
           (apply
            #'max (mapcar
                   #'(lambda (j) (/ (abs (send j :joint-torque))
                                    (send j :max-joint-torque)))
                   (flatten (send *ri* :robot :arms :joint-list))))))
      (pprint mtrq)
      (when (> mtrq 0.8)
        (ros::ros-info "TORQUE INTERRUPT ~A" mtrq)
        (send *ri* :cancel-all-motion :torque)
        (speak-jp "中断します")
        (send *ri* :start-mannequin-mode :rarm)
        (return-from wait-torque t)
        ))
    (ros::sleep)))

(defun voice-callback (msg)
  (warn "received: ~A~%" (send msg :texts))
  (when (memq t (mapcar #'(lambda (s) (substringp "ストップ" s)) (send msg :texts)))
    (ros::ros-info "VOICE-INTERRUPT")
    (send *ri* :cancel-all-motion)
    (speak-jp "中断します")
    (send *ri* :start-mannequin-mode :rarm)
    (read-char)
    (send-to-server)
    ))

(defun wait-voice ()
  (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'voice-callback)
  (do-until-key
      (ros::spin-once)
    (ros::sleep)))

(defun send-to-server ()
  (warn "send-to-server~%")
  ;; make modified avs-tms
  (let* ((ftime (send (send (send *last-feedback* :actual) :time_from_start)))
         (gtime (send (send (elt (send (send *last-goal* :trajectory) :points) 0) :time_from_start)))
         (f-gtime (ros::time- gtime ftime))
         (avs (eus->ros
              (list
               (list
                (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
                (apply-trajectory_point (send *last-goal* :joint_names)
                                        (send *last-goal* :points)
                                        *pr2*))
               (list
                (/ (send ftime :to-nsec) 1000)
                (/ (send f-gtime :to-nsec) 1000))))))

  (send *ri* :stop-mannequin-mode :rarm)
  (ros::publish "/modify_avs"
                (instance local_pr2_demo::AngleVectorSequence :init
                          :id (ros::get-param *angle-vector-flag*)
                          :data avs))))

(defun main ()
  (ros::rate 10)
  (ros::advertise "/modify_avs" local_pr2_demo::AngleVectorSequence)
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/feedback_throttle" control_msgs::FollowJointTrajectoryActionFeedback #'feedback-cb)
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal #'goal-cb)
  (pr2-init)
  (wait-voice)
;;  (wait-torque)
;;  (read-char)
;;  (send-to-server)
  )

(main)
