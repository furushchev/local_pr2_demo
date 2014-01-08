#!/usr/bin/env roseus
(ros::roseus "test_server")
(ros::load-ros-manifest "local_pr2_demo")
(require :test-util "test-util.l")

;; database
(defparameter *hash* (make-hash-table))

(defun geth (k)
  (when (stringp k) (setq k (read-from-string (string-upcase k))))
  (gethash k *hash*))

(defun seth (k v)
  (when (stringp k) (setq k (read-from-string (string-upcase k))))
  (setf (gethash k *hash*) v))

(defun register (msg)
  (ros::ros-info "registered: ~A -> ~A" () (send msg :data))
  (seth (ros::get-param *angle-vector-flag*) (send msg :data)))

(defun service-callback (req)
  (warn "service-callback~%")
  (let* ((req-avs (send req :data))
         (res (send req :response))
         (db-res (geth (send req-avs :id)))
         (res-avs (instance local_pr2_demo::AngleVectorSequence :init)))
    (ros::ros-info "~A: ~A" (send req-avs :id) db-res)
    (send res-avs :id (send req-avs :id))
    (if db-res
        (send res-avs :data db-res)
        (send res-avs :data (send req-avs :data)))
    (send res :data res-avs)))

;; main loop
(defun main ()
  (ros::advertise-service "/read_avs" local_pr2_demo::AngleVectorSequenceService #'service-callback)
  (ros::subscribe "/modify_avs" local_pr2_demo::AngleVectorSequence #'register)
  (ros::rate 100)
  (ros::ros-info "initialized")
  (when (ros::ok)
    (do-until-key
        (ros::spin-once)
      (ros::sleep)))
  )

(main)
