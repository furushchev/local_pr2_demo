#!/usr/bin/env roseus

(ros::roseus "test_demo")
(ros::load-ros-manifest "local_pr2_demo")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :test-client "test-client.l")

(load "template_grasp_eus.l")

(defparameter *use-arm* :larm)
(defparameter *type* "georgia")

(defun demo ()
;;  (move-to-spot "/eng2/7f/room73B2-center")
;;  (pr2-tuckarm-pose)
  (move-to-spot "/eng2/7f/room73B2-table-side1"
                :angle-vector (pr2-pick-tray-pose)
                :clear-costmap t
                :tilt-enable nil)
  (pick-tray)
  (change-inflation-range 0.4)
  (move-to-spot "/eng2/7f/room73B2-sink-front0"
                :tilt-enable nil
                :clear-costmap t)
  ;; INTERRUPT
  (unix:sleep 5)
  (move-to-spot "/eng2/7f/room73B2-counter-side"
                :clear-costmap t
                :tilt-enable nil)
  (place-tray :table-height 850
              :go-forward 300)
  (change-inflation-range 1.0)
  (pr2-tuckarm-pose)
  (move-to-spot "/eng2/7f/room73B2-fridge-front"
                :clear-costmap t
                :tilt-enable t)
  (pr2-reset-pose)
  (open-fridge-door :use-arm *use-arm*)
  (grasp-can :use-arm *use-arm*)
  (close-fridge :use-arm *use-arm*)
  (move-to-spot "/eng2/7f/room73B2-table-front2"
                :clear-costmap t
                :tilt-enable nil)
  (hand-over *use-arm* :wait-shock t)
  (move-to-spot "/eng2/7f/room73B2-counter-side"
                :clear-costmap t
                :tilt-enable nil)
  (pick-tray)
  (move-to-spot "/eng2/7f/room73B2-sink-front1"
                :tilt-enable nil
                :clear-costmap t)
  (place-tray :table-height 900)
  ;; TRAY TASK END
  (pr2-reset-pose)
  )

(defaction action-pick-can (&key (arm :rarm)
                          (can-coords (make-coords :pos #f(500 0 800))))
  (send *ri* :stop-grasp arm :wait t)
  (send *pr2* :head :neck-p :joint-angle 40)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (let ((pr2c (send (send *pr2* :worldcoords) :copy-coords)))
    (send *pr2* :reset-coords)
    (let ((avs
           (list
#|            (send *pr2* arm :inverse-kinematics
                  can-coords
                  :rotation-axis t)
            (send *pr2* :rarm :move-end-pos #f(200 0 0)))))|#
#|            (float-vector 50.0691 60.0004 73.9992 69.9958 -119.999 19.9995 -29.9999 179.999 -24.7351 16.4856 -86.5539 -121.542 189.001 -14.9195 83.2877 -0.006113 41.148)
            #f(50.0694 60.0004 73.9992 69.9958 -119.999 19.9995 -29.9999 179.999 -20.403 15.8313 -75.1796 -121.176 311.167 -67.0206 108.725 -0.024113 41.88)
            #f(50.0694 60.0004 73.9992 69.9958 -119.999 19.9995 -29.9999 179.999 -16.5315 1.36321 -93.0312 -119.584 451.564 -90.0872 76.6392 0.011887 41.886)
            #f(50.0694 60.0004 73.9992 69.9958 -119.999 19.9995 -29.9999 179.999 -13.5152 5.10989 -91.5704 -114.275 541.126 -97.2193 92.3269 -0.024113 41.886)
            (float-vector 50.069 60.0004 73.9992 70.005 -119.999 19.9995 -29.9999 179.999 -11.4251 -7.92351 -105.067 -94.0609 188.706 -75.1649 107.391 -0.012113 41.148))))
|#
            #f(50.0691 60.0004 73.9992 69.9958 -119.999 19.9962 -29.9999 179.999 -20.7925 31.2397 -98.6816 -121.542 -43.8581 -84.4758 227.541 0.005887 -1.134)
            #f(50.0691 60.0004 73.9992 70.005 -119.999 19.9995 -29.9974 180.002 -14.7692 2.95785 -101.337 -121.542 93.4705 -103.389 249.51 -0.018113 -1.14)
            #f(50.069 60.0004 73.9992 69.9958 -119.999 20.0061 -29.9999 179.999 -10.3468 -3.19776 -101.043 -112.285 183.662 -99.9416 282.982 0.023887 -1.14)
            #f(50.069 60.0004 74.0041 69.9958 -119.999 19.9962 -29.9999 179.999 -11.3586 -0.934238 -98.4979 -92.1117 184.557 -78.1837 281.382 -0.036113 -1.14))))
      (send-ri-angle-vector-sequence avs (list 2000 1000 1000 1500))
      (when (memq nil (send *ri* :wait-interpolation))
          (send *pr2* :transform pr2c)
          (return-from action-pick-can :interrupt))
      (send *ri* :start-grasp arm)
      (send *pr2* :transform pr2c)
  )))

(defparameter *cup-geometry* (instance geometry_msgs::Point :init
                                       :x 0.6 :y 0.0 :z 0.8))

(defun check-wait (m)
  (if (string= (car (send m :texts)) "waitwait")
      (setq *wait* t)))

(setq *wait* nil)
(defun demo2 (&optional (resume nil))
  (when (not resume)
    (pr2-tuckarm-pose)
    (move-to-spot "/eng2/7f/room73B2-table-side1"
                  :clear-costmap t))
  (pr2-reset-pose)
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
;;  (unix:sleep 5)
  (when (eq (action-template-grasp-eus *cup-geometry* :arm :rarm) :interrupt)
      (while (eq *wait* nil)
        (ros::spin-once)
        (ros::sleep))
      (demo2 t))
  (send *pr2* :rarm :move-end-pos #f(-200 0 100))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (move-to-spot "/eng2/7f/room73B2-sink-front1"
                :clear-costmap t
                :tilt-enable nil)
  (send *pr2* :rarm :inverse-kinematics
        (make-coords :pos #f(750 0 890))
        :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :arm :wait t)
  (pr2-tuckarm-pose)
  (move-to-spot "/eng2/7f/room73B2-center"
                :clear-costmap t)
  )

(defun match-stringp (substr str-lst)
  (dolist (s str-lst)
    (if (substringp substr s)
        (return-from match-stringp s)))
  nil)

(defun main ()
  (ros::rate 10)
  (ros::subscribe "/Tablet/voice"
                  jsk_gui_msgs::VoiceMessage
                  #'(lambda (msg)
                      (let ((s (send msg :texts)))
                        (cond
                          ((match-stringp "片付け" s)
                           (demo))
                          ((match-stringp "はこんで" s)
                           (demo2))
                          ((match-stringp "しゃべれよ" s)
                           (speak-jp "しゃべるよ!"))
                          ))))
  (pr2-init)
  (ros::ros-info "waiting for tablet voice...")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

;;(main)