#!/usr/bin/env roseus
(ros::roseus "test_interrupter")
(ros::load-ros-manifest "local_pr2_demo")
(ros::roseus-add-msgs "sensor_msgs")

(load "package://jsk_demo_common/euslisp/attention-observation.l")
(require :test-util "test-util.l")

(defparameter *last-goal-id* nil)
(defparameter *last-goal* nil)
(defparameter *last-feedback* nil)
(defparameter *current-frame* 0)
(defparameter *msec-per-frame* 100)
(defparameter *is-mannequin* nil)
(defparameter *modified* nil)
(defparameter *before-pose* nil)

(defun feedback-cb (msg)
  (let* ((status (send msg :status))
         (goal-id (send status :goal_id))
         (feedback (send msg :feedback)))
    (warn "feedback: ~A~%" (length *last-feedback*))
    (push feedback *last-feedback*)
    ))

(defun goal-cb (msg)
  (let* ((goal-id (send msg :goal_id))
         (goal (send msg :goal)))
    (ros::ros-info "goal-cb (~A)~%" (send goal-id :id))
    (setq *last-feedback* nil)
    (setq *last-goal-id* (send goal-id :id))
    (setq *last-goal* goal)))

(defun exec-interrupt ()
    (send *ri* :cancel-all-motion)
    (send *ri* :start-mannequin-mode :rarm)
    (read-char)
    (send-to-server)
    )

(defun pwd (f)
  (when *is-mannequin*
    (ros::ros-info "mannequin!")
    (return-from pwd nil))
  (when (>= *current-frame* (length *last-feedback*))
      (setq *current-frame* (1- (length *last-feedback*)))
      (ros::ros-info "current-frame reset to ~A" *current-frame*))
  (ros::ros-info "pwd ~A -> ~A" *current-frame* (inc *current-frame* f))
  (send-joint_traj_feedback (elt *last-feedback* *current-frame*))
  )

(defun fwd (f)
  (when *is-mannequin*
    (ros::ros-info "mannequin!")
    (return-from fwd nil))
  (when (< *current-frame* 0)
      (ros::ros-info "*current-frame* reset to 0")
      (setq *current-frame* 0))
  (ros::ros-info "fwd ~A -> ~A" *current-frame* (dec *current-frame* f))
  (send-joint_traj_feedback (elt *last-feedback* *current-frame*))
  )

(defun stop ()
  (send *ri* :cancel-all-motion)
  (unsubscribe-joints)
  ;; (nreverse *last-feedback*)
  (ros::ros-info "feedback length: ~A" (length *last-feedback*))
  )

(defun wait-button ()
  (ros::advertise-service "/Tablet/button" local_pr2_demo::Int64Int64
                          #'(lambda (req)
                              (let ((res (send req :response)))
                                (ros::ros-info "pressed: ~A" (send req :data))
                                (case (send req :data)
                                  (0 ;; stop
                                   (progn
                                     (stop)
                                     (send res :data (length *last-feedback*))
                                     ))
                                  (1 ;; pwd
                                   (progn
                                     (pwd 1)
                                     (send res :data *current-frame*)))
                                  (2 ;; fwd
                                   (progn
                                     (fwd 1)
                                     (send res :data *current-frame*)))
                                  (3 ;; send
                                   (progn
                                     (send-to-server)
                                     (subscribe-joints)
                                     (send res :data 0)))
                                  (4 ;; pause/play
                                   (progn
                                     (if (not *is-mannequin*) ;; pause
                                         (progn
                                           (when *goal-mode*
                                             (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
                                             (setq *before-pose* *pr2*))
                                           (send *ri* :start-mannequin-mode :rarm))
                                       (progn ;; play
                                         (send *ri* :stop-mannequin-mode :rarm)
                                         (when *goal-mode* ;; goal mode
                                           (send-to-server-goal)
                                           (send res :data 0)
                                           (return-from wait-button res)
                                           )
                                         ;; avoid mode
                                         (push (cons
                                                (send *ri* :state :potentio-vector)
                                                (* 1000.0 (send (send (send (elt *last-feedback* *current-frame*) :actual) :time_from_start) :to-sec)))
                                                *modified*)
                                         ))
                                     (setq *is-mannequin* (null *is-mannequin*))
                                     (send res :data 0)))
                                  (5 ;; pwd10
                                   (progn
                                     (pwd 10)
                                     (send res :data *current-frame*)))
                                  (6 ;; fwd10
                                   (progn
                                     (fwd 10)
                                     (send res :data *current-frame*)))
                                  (7 ;; reset
                                   (progn
                                     (pr2-reset-pose)
                                     (send res :data 0)))
                                  (8 ;; toggle goal mode/ avoid mode
                                   (progn
                                     (setq *goal-mode* (null *goal-mode*))
                                     (if *goal-mode*
                                         (ros::ros-info "avoid -> goal")
                                       (ros::ros-info "goal -> avoid"))
                                     ))
                                  (t ;; slider
                                   (let ((diff (- (/ (send req :data) 100) *current-frame*)))
                                     (if (> diff 0)
                                         (pwd diff)
                                       (fwd (- diff)))
                                     (send res :data *current-frame*)))
                                  )
                                res)))
  (ros::ros-info "waiting button")
  (do-until-key
   (ros::spin-once)
   (ros::sleep)))

(defun send-joint_traj_feedback (msg)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (apply-trajectory_point (send msg :joint_names)
                          (send msg :actual)
                          *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500))

(defun traj_goal->avs (goal robot)
  (let* ((traj (send goal :trajectory))
         (names (send traj :joint_names))
         (points (send traj :points))
         avs)
    (dolist (p points)
      (apply-trajectory_point names p robot)
      (push (cons (send robot :angle-vector)
                  (* 1000.0 (send (send p :time_from_start) :to-sec)))
            avs))
    (nreverse avs)))

(defun time_from_start->tms (src)
  (let ((sorted-avs (sort (copy-object src) #'(lambda (x y) (< (cdr x) (cdr y)))))
        ptm avs tms)
    (dolist (av-tfs sorted-avs)
      (cond
       ((eq av-tfs (car sorted-avs))
        (setq ptm (cdr av-tfs))
        (push (car av-tfs) avs)
        (push ptm tms))
       (t
        (push (car av-tfs) avs)
        (push (- (cdr av-tfs) ptm) tms)
        (setq ptm (cdr av-tfs)))))
    (list (nreverse avs)
          (nreverse tms))
    ))

(defun apply-diff-pose-to-traj_goal (goal robot)
  (warn "apply-diff-pose-to-traj_goal~%")
  (let (diff-pose)
    (setq diff-pose (;; TODO: *before-pose* と現在の目標手先姿勢との差分をとって、それをgoal_trajの各々に差分の分だけずらしたものを計算してserverへ送る
                     0))))

(defun send-to-server ()
  (warn "send-to-server~%")
  (let (ret-goal diff-tms last-time)
;;    (setq ret-goal (apply-joint_traj_goal *last-goal*))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq ret-goal (time_from_start->tms (append (traj_goal->avs *last-goal* *pr2*) *modified*)))
    (ros::ros-info "publish: ~A" ret-goal)
    (ros::publish "/modify_avs" (eus->ros ret-goal :id *last-goal-id*))
    (setq *time-from-starts* nil *current-frame* 0)
    ))

(defparameter *joy-accum* 0.0)
(defun joy-cb (msg)
  (let ((scratch (aref (send msg :axes) 0))
        (gain (aref (send msg :axes) 3))
        (play/pause-button (aref (send msg :buttons) 6))
        (stop-button (aref (send msg :buttons) 7))
        (reset-button (aref (send msg :buttons) 9))
        (mode-button (aref (send msg :buttons) 10))
        (send-button (aref (send msg :buttons) 8))
        )

    (cond
     ((eq stop-button 1)
      (stop))
     ((eq send-button 1)
      (send-to-server)
      (subscribe-joints))
     ((eq play/pause-button 1)
      (if (not *is-mannequin*) ;; pause
          (progn
            (when *goal-mode*
              (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
              (setq *before-pose* *pr2*))
            (send *ri* :start-mannequin-mode :rarm))
        (progn ;; play
          (send *ri* :stop-mannequin-mode :rarm)
          (when *goal-mode* ;; goal mode
            (send-to-server-goal)
            (return-from joy-cb t)
            )
          ;; avoid mode
          (push (cons
                 (send *ri* :state :potentio-vector)
                 (* 1000.0 (send (send (send (elt *last-feedback* *current-frame*) :actual) :time_from_start) :to-sec)))
                *modified*)
          ))
      (setq *is-mannequin* (null *is-mannequin*)))
     ((eq reset-button 1)
      (pr2-reset-pose))
     ((eq mode-button 1)
      (setq *goal-mode* (null *goal-mode*))
      (if *goal-mode*
          (ros::ros-info "avoid -> goal")
        (ros::ros-info "goal -> avoid")))
    ((not (eq scratch 0))
     (inc *joy-accum* (* scratch gain 10))
     (when (> (floor *joy-accum*) 0)
       (fwd 1)
       (setq *joy-accum* 0.0))
     (when (< (1+ (floor *joy-accum*)) 0)
       (pwd 1)
       (setq *joy-accum* 0.0))))
  ))

(defun subscribe-joints ()
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/feedback_throttle" control_msgs::FollowJointTrajectoryActionFeedback #'feedback-cb)
  (ros::subscribe "/r_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal #'goal-cb)
  )

(defun unsubscribe-joints ()
  (ros::unsubscribe "/r_arm_controller/follow_joint_trajectory/feedback_throttle")
  (ros::unsubscribe "/r_arm_controller/follow_joint_trajectory/goal"))

(defun main ()
  (ros::rate 10)
  (ros::advertise "/modify_avs" local_pr2_demo::AngleVectorSequence)
  (ros::subscribe "/vestax_spin2/joy" sensor_msgs::Joy #'joy-cb)
  (subscribe-joints)
  (pr2-init)
  (wait-button)
  )

(main)
