#!/usr/bin/env roseus

(ros::roseus "test_demo")
(ros::load-ros-manifest "local_pr2_demo")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :test-client "test-client.l")

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
                          (can-coords (make-coords :pos #f(600 0 800))))
  (send *ri* :stop-grasp arm :wait t)
  (let ((pr2c (send (send *pr2* :worldcoords) :copy-coords)))
    (send *pr2* :reset-coords)
    (let ((avs
           (list
            (send *pr2* arm :inverse-kinematics
                  (make-coords :pos #f(600 0 800))
                  :rotation-axis t)
            (send *pr2* :rarm :move-end-pos #f(200 0 0)))))
      ;;    (send *ri* :angle-vector-sequence avs (list 3000 1000)))
      (send-ri-angle-vector-sequence avs (list 3000 1000))
      (if (memq nil (send *ri* :wait-interpolation))
          (send *pr2* :transform pr2c)
          (return-from pick-can nil))
      (send *ri* :start-grasp arm)
      (send *pr2* :transform pr2c)
  )))


(defun demo2 ()
  (pr2-tuckarm-pose)
  (move-to-spot "/eng2/7f/room73B2-table-side1"
                :clear-costmap t)
  (pr2-reset-pose)
  (action-pick-can)
  (send *pr2* :rarm :move-end-pos #f(-200 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (move-to-spot "/eng2/7f/room73B2-trashbox-front"
                :clear-costmap t
                :tilt-enable nil)
  (send *pr2* :rarm :inverse-kinematics
        (make-coords :pos #f(750 0 650))
        :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :wait t)
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
                          ((match-stringp "ゴミ捨て" s)
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