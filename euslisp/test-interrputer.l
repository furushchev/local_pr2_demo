#!/usr/bin/env roseus
(ros::roseus "test_interrupter")
(ros::load-ros-manifest "local_pr2_demo")

(load "package://jsk_demo_common/euslisp/attention-observation.l")
(require :test-util "test-util.l")

(defparameter *last-goal-id* nil)
(defparameter *last-goal* nil)
(defparameter *last-feedback* nil)
(defparameter *current-frame* 0)
(defparameter *msec-per-frame* 100)
(defparameter *pausing* nil)
(defparameter *avs* nil)
(defparameter *tms* nil)

(defun feedback-cb (msg)
  (let* ((status (send msg :status))
         (goal-id (send status :goal_id))
         (feedback (send msg :feedback)))
;;    (warn "feedback")
    (push feedback *last-feedback*)
    ))

(defun goal-cb (msg)
  (warn "goal-cb~%")
  (let* ((goal-id (send msg :goal_id))
         (goal (send msg :goal)))
    (setq *last-feedback* nil)
    (setq *last-goal-id* (send goal-id :id))
    (setq *last-goal* goal)))
#|
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
|#

(defun exec-interrupt ()
    (send *ri* :cancel-all-motion)
;;    (speak-jp "中断します")
    (send *ri* :start-mannequin-mode :rarm)
    (read-char)
    (send-to-server)
    )
#|
(defun voice-callback (msg)
  (warn "received: ~A~%" (send msg :texts))
  (when (memq t (mapcar #'(lambda (s) (substringp "ストップ" s)) (send msg :texts)))
    (ros::ros-info "VOICE-INTERRUPT")
    (exec-interrupt)
    ))

(defun wait-voice ()
  (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'voice-callback)
  (do-until-key
      (ros::spin-once)
    (ros::sleep)))
|#

(defun pwd (f)
  (when (not *pausing*)
    (ros::ros-info "not pausing")
    (return-from pwd nil))
  (ros::ros-info "pwd ~A -> ~A" *current-frame* (inc *current-frame* f))
  (if (< *current-frame* (length *last-feedback*))
      (setq *current-frame* (length (1- *last-feedback*))))
  (send-joint_traj_feedback (elt *last-feedback* *current-frame*))
  )

(defun fwd (f)
  (when (not *pausing*)
    (ros::ros-info "not pausing")
    (return-from fwd nil))
  (ros::ros-info "fwd ~A -> ~A" *current-frame* (dec *current-frame* f))
  (if (< *current-frame* 0)
      (setq *current-frame* 0))
  (send-joint_traj_feedback (elt *last-feedback* *current-frame*))
  )

(defun wait-button ()
  (ros::subscribe "/Tablet/button" std_msgs::UInt8
                  #'(lambda (m)
                      (case (send m :data)
                        (0 ;; stop
                         (progn
                           (send *ri* :cancel-all-motion)
;;                           (nreverse *last-feedback*)
                           (ros::ros-info "feedback length: ~A" (length *last-feedback*))
                           ))
                        (1 ;; pwd
                         (pwd 1)
                           )
                        (2 ;; fwd
                         (fwd 1))
                        (3 ;; send
                         (send-to-server))
                        (4 ;; pause
                         (progn
                           (if *pausing*
                               (send *ri* :start-mannequin-mode :rarm)
                             (progn
                               (send *ri* :stop-mannequin-mode :rarm)
                               (push (send *ri* :state :potentio-vector) *avs*)
                               (push *current-frame* *tms*)
                               ))
                           (setq *pausing* (null *pausing*))))
                        (5 ;; pwd10
                         (pwd 10))
                        (6 ;; fwd10
                         (fwd 10))
                        )))
  (do-until-key
   (ros::spin-once)
   (ros::sleep)))

(defun send-joint_traj_feedback (msg)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (apply-trajectory_point (send msg :joint_names)
                          (send (send msg :actual) :positions)
                          *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500))

(defun apply-joint_traj_goal (msg)
  (let ((traj (send msg :trajectory)))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (list (apply-trajectory_point (send traj :joint_names)
                                  (send traj :points)
                                  *pr2*)
          (/ (send (send (car (send traj :points)) :time_from_start) :to-nsec) 1000))
          ))

(defun send-to-server ()
  (warn "send-to-server~%")
  (let (ret-goal diff-tms last-time)
    (setq ret-goal (apply-joint_traj_goal *last-goal*))
    (push (car ret-goal) *avs*)
    (setq last-time (- (cadr ret-goal) (* *msec-per-frame* (car *tms*))))
    (nreverse *avs*)
    (append *tms* (cons 0 nil))
    (dotimes (i (1- (length *tms*)))
      (push (* *msec-per-frame* (- (elt *tms* i) (elt *tms* (1+ i)))) diff-tms))
    (append diff-tms (cons last-time nil))
    (ros::ros-info "published: ~A ~A" *avs* diff-tms)
    (ros::publish "/modify_avs"
                (eus->ros (list *avs* diff-tms))))
  (setq *avs* nil *tms* nil *current-frame* 0))

#|
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
|#

(defun main ()
  (ros::rate 10)
  (ros::advertise "/modify_avs" local_pr2_demo::AngleVectorSequence)
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/feedback_throttle" control_msgs::FollowJointTrajectoryActionFeedback #'feedback-cb)
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal #'goal-cb)
  (pr2-init)
  (wait-button)
;;  (wait-voice)
;;  (wait-torque)
;;  (read-char)
;;  (send-to-server)
  )

(main)
