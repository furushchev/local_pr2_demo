#!/usr/bin/env roseus
(ros::roseus "test_client")
(ros::load-ros-manifest "local_pr2_demo")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")

(require :test-util "test-util.l")

(defmacro defaction (name arg-lst &rest form)
  `(defun ,name ,arg-lst
     (setq *function-name* (string ',name))
     (setq *angle-vector-number* -1)
     ,@form))

(defun send-ri-angle-vector-sequence (avs tms)
  (if (not (and (boundp '*function-name*) (boundp '*angle-vector-number*)))
      (ros::ros-error "this function must be used in function defined by defaction"))
  (let* ((id (format nil "~A_~A" *function-name* (inc *angle-vector-number*)))
         (req (instance local_pr2_demo::AngleVectorSequenceServiceRequest :init
                        :data (eus->ros (list avs tms) :id id)))
         (res (ros::service-call "/read_avs" req)))
    (ros::set-param *angle-vector-flag* id)
    (if res
        (let ((avs-tms (ros->eus (send res :data))))
          (ros::ros-info "received: ~A" avs-tms)
          (send *ri* :angle-vector-sequence (car avs-tms) (cadr avs-tms)))
        (send *ri* :angle-vector-sequence avs tms))))

(defun send-ri-angle-vector (av &optional (time 3000))
  (send-ri-angle-vector-sequence (list av) (list time)))


(defaction pick-can (&optional (arm :rarm)
                               (can-coords (make-coords :pos #f(680 0 800)))
                               )
    (send *ri* :stop-grasp arm)
    (send *pr2* arm :inverse-kinematics
          can-coords
          :rotation-axis t)
    (send *pr2* arm :move-end-pos #f(-150 0 0))
    (send-ri-angle-vector (send *pr2* :angle-vector) 20000)
    (if (memq nil (send *ri* :wait-interpolation))
        (return-from pick-can nil))
    (send *pr2* arm :move-end-pos #f(170 0 0))
    (send-ri-angle-vector (send *pr2* :angle-vector) 10000)
    (if (memq nil (send *ri* :wait-interpolation))
        (return-from pick-can nil))
    (send *ri* :start-grasp arm)
    )

;; main loop
(defun main ()
  (pr2-init)
  (pick-can))

;;(main)
