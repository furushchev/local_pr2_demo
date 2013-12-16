#!/usr/bin/env roseus

(require :state-machine "package://roseus_smach/src/state-machine.l")
;;(load "package://jsk_demo_common/euslisp/attention-observation.l")


(defparameter *move-to-state* 'not-executed)

(defun move-to-fridge-front (args)
  (format t "move-to-fridge-front~%")
  t)

(defun open-fridge-door (args)
  (format t "open-fridge-door~%")
  t)

(defun grasp-can (args)
  (format t "grasp-can~%")
  t)

(defun pass-can (args)
  (format t "pass-can~%")
  t)

(defun smach-fridge ()
  (let ((sm (instance state-machine :init)))
	(send sm :add-node (instance state :init "MOVE " 'move-to-fridge-front))
	(send sm :add-node (instance state :init "OPEN-DOOR" 'open-fridge-door))
	(send sm :add-node (instance state :init "GRASP-CAN" 'grasp-can))
	(send sm :add-node (instance state :init "CLOSE-DOOR" 'close-fridge-door))
	(send sm :add-node (instance state :init "PASS" 'pass-can))

	(send sm :goal-state :END)

	(send sm :start-state "MOVE ")

	(send sm :add-transition "MOVE " "OPEN-DOOR" t)
	(send sm :add-transition "OPEN-DOOR" "GRASP-CAN" t)
	(send sm :add-transition "GRASP-CAN" "CLOSE-DOOR" t)
	(send sm :add-transition "CLOSE-DOOR" "PASS" t)
	(send sm :add-transition "PASS" :END t)
	sm))

(defparameter *decision* t)
(defun decision-make (args)
  (format t "decision-make~%")
  *decision*)

(defun smach-all ()
  (let ((sm-root (instance state-machine :init))
		(sm-tray (smach-tray))
		(sm-fridge (smach-fridge)))
	(send sm-root :add-node (instance state :init "START" 'decision-make))
	(send sm-root :add-node (instance state :init "TRAY" sm-tray))
	(send sm-root :add-node (instance state :init "FRIDGE" sm-fridge))
	(send sm-root :start-state "START")
	(send sm-root :goal-state :END)
	(send sm-root :add-transition "START" "TRAY" t)
	(send sm-root :add-transition "START" "FRIDGE" nil)
	(send sm-root :add-transition "TRAY" "FRIDGE" :tray->fridge)
	(send sm-root :add-transition "FRIDGE" "TRAY" :fridge->tray)
	(send sm-root :add-transition "TRAY" :END t)
	sm-root))

(defun pick-tray (args)
  (format t "pick-tray~%")
  t)

(defun move-to-sink-internal (status)
  (format t "move-to-sink-internal~%")
  t)

(defun check-pos-diff (pos)
  (format t "check-pos-diff~%")
  (let (start-pos goal-pos)
  (read-char)))

(defun move-to-sink (args)
  (format t "move-to-sink~%")
  (let ((move-to-res (move-to-sink-internal *move-to-state*)))
	(read-char))
	   
  (case (read-char)
	(49 (progn
		  (setq *decision* nil)
		  (send *sm* :reset-state)))
	(t t))
  t)

(defun move-to-sink-fail-start (args)
  (format t "move-to-sink-fail-start~%"))

(defun move-to-sink-fail-save (args)
  (format t "move-to-sink-fail-save~%"))

(defun move-to-sink-fail-goal (args)
  (format t "move-to-sink-fail-goal~%"))

(defun place-tray (args)
  (format t "place-tray~%")
  t)

(defun smach-tray ()
  (let ((sm (instance state-machine :init)))

	(send sm :add-node (instance state :init "PICK" 'pick-tray))
	(send sm :add-node (instance state :init "MOVE" 'move-to-sink))
	(send sm :add-node (instance state :init "PLACE" 'place-tray))

	(send sm :goal-state :end)

	(send sm :start-state "PICK")
	
	(send sm :add-transition "PICK" "MOVE" t)
	(send sm :add-transition "MOVE" "PLACE" t)
	(send sm :add-transition "PLACE" :end t)
	sm))


(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

(ros::roseus "solve_72b2_demo")

(defun debug-exec-smach (sm)
  (let ((insp (instance state-machine-inspector :init sm))
		(mydata (list (cons 'count 0))))
	(unix:sleep 1)
	(send sm :reset-state)
	(send insp :publish-structure)
	(send insp :publish-status mydata)
	(while (not (send sm :goal-reached))
	  (unix:sleep 1)
	  (ros::spin-once)
	  (send insp :publish-status mydata)
	  (send sm :execute mydata :step -1))
	(send sm :active-state)))

(setq *sm* (smach-all))

;;(debug-exec-smach *sm*)
;;(exec-smach-with-interrupt (smach-all))