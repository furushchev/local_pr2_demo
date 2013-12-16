#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_gui_msgs")

(load "package://jsk_demo_common/euslisp/attention-observation.l")
(load "package://pr2eus/speak.l")

(ros::roseus "tablet_interrupt")
(ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage
                #'(lambda (msg)
                    (let ((str-lst (send msg :texts)))
                      (format t "received: ~A~%" str-lst)
                      (when (memq t (mapcar #'(lambda (s) (substringp "ジョージア" s)) str-lst))
                        (speak-jp "うけたまわりました。")
                        (ros::ros-info "Interrupting with tablet voice message...")
                        (send *ri* :cancel-all-motion)
                        )
                      (when (memq t (mapcar #'(lambda (s) (substringp "しゃべれよ" s)) str-lst))
                        (speak-jp "しゃべるよ！"))
                      (when (memq t (mapcar #'(lambda (s) (substringp "こんにちは" s)) str-lst))
;;                        (speak-jp "気安く、話しかけんな。こっちは遊びじゃないんや。"))
                        (speak-jp "やあ。"))
                    )))

(defun setup ()
  (pr2-init)
  (ros::rate 30)
  (ros::ros-info "Tablet waiting...")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

(setup)
