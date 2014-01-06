#!/usr/bin/env roseus

(require :action-73b2-demo "action-73b2-demo.l")

(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")


(defparameter *fail-state* nil)

(defun smach-tray ()
  (let ((sm (instance state-machine :init)))
	(send sm :arg-keys '*fail-state*)
	(send sm :add-node (instance state :init "start" 'start))
	(send sm :add-node (instance state :init "resume?" 'resume-tray))
	(send sm :add-node (instance state :init "(move-to table-side)" '(lambda (&rest args) (move-to 'table-side))))
	(send sm :add-node (instance state :init "(pick-tray)" 'pick-tray-table))
	(send sm :add-node (instance state :init "(move-to sink-front)" '(lambda (&rest args) (move-to 'sinkfront))))
	(send sm :add-node (instance state :init "(place-tray)" 'place-tray-table))

	(send (send sm :node "resume?") :remap-list '((fail-state . *fail-state*)))

	(send sm :start-state "start")
	(send sm :goal-state :goal)

	(send sm :add-transition "start" "resume?" t)
	(send sm :add-transition "resume?" "(move-to table-side)" t)
	(send sm :add-transition "(move-to table-side)" "(pick-tray)" t)

	(send sm :add-transition "(pick-tray)" "(move-to sink-front)" t)
	(send sm :add-transition "(move-to sink-front)" "(place-tray)" t)
	(send sm :add-transition "(place-tray)" :goal t)
	sm))

(defun add-fridge (sm)
  (send sm :add-node (instance state :init "resume? " 'resume-fridge))
  (send sm :add-node (instance state :init "(move-to fridgefront)" '(lambda (&rest args) (move-to 'fridgefront))))
  (send sm :add-node (instance state :init "(open-door)" 'open-door))
  (send sm :add-node (instance state :init "(grasp-can)" 'grasp-can-fridge))
  (send sm :add-node (instance state :init "(close-door)" 'close-door))
  (send sm :add-node (instance state :init "(pass-to-human)" 'pass-to-human))

  (send sm :add-transition "resume? " "(move-to fridgefront)" t)
  (send sm :add-transition "(move-to fridgefront)" "(open-door)" t)
  (send sm :add-transition "(open-door)" "(grasp-can)" t)
  (send sm :add-transition "(grasp-can)" "(close-door)" t)
  (send sm :add-transition "(close-door)" "(pass-to-human)" t)
  )

(defun add-fail (sm)
  (send sm :add-node (instance state :init "(revert-to-start)" 'revert-to-start))
  (send sm :add-node (instance state :init "(place-tray-temporally)" 'place-tray-temporally))
  (send sm :add-node (instance state :init "(continue-move-to)" 'continue-move-to))

  (send (send sm :node "(revert-to-start)") :remap-list '((fail-state . *fail-state*)))
  (send (send sm :node "(place-tray-temporally)") :remap-list '((fail-state . *fail-state*)))
  (send (send sm :node "(continue-move-to)") :remap-list '((fail-state . *fail-state*)))
  (send (send sm :node "(pick-tray)") :remap-list '((fail-state . *fail-state*)))
  
  (send sm :add-transition "(move-to sink-front)" "(revert-to-start)" :start)
  (send sm :add-transition "(move-to sink-front)" "(place-tray-temporally)" :temp)
  (send sm :add-transition "(move-to sink-front)" "(continue-move-to)" :goal)

  (send sm :add-transition "resume?" "(move-to table-side)" :start)
  (send sm :add-transition "resume?" "(pick-tray)" :temp)
  (send sm :add-transition "resume?" :goal :goal)
  )

(defun add-resume (sm)
  (send sm :add-transition "(revert-to-start)" "resume? " t)
  (send sm :add-transition "(place-tray-temporally)" "resume? " t)
  (send sm :add-transition "(continue-move-to)" "resume? " t)

;;  (send sm :add-transition "(revert-to-start)" "(move-to table-side)" :resume)
;;  (send sm :add-transition "(place-tray-temporally)" "(move-to table-side)" :resume)
;;  (send sm :add-transition "(continue-move-to)" "(move-to sink-front)" :resume)

  (send sm :add-transition "(pass-to-human)" "resume?" t)
  )

(ros::roseus "hoge")
(defun init ()
  (setq *sm* (smach-tray))
  (setq *insp* (instance state-machine-inspector :init *sm*))
  (send *insp* :publish-structure))

(defun change-smach ()
;;  (read-char)
  (unix:sleep 3)
  (add-fridge *sm*)
  (send *insp* :publish-structure)

;;  (read-char)
  (unix:sleep 3)
  (add-fail *sm*)
  (send *insp* :publish-structure)

;;  (read-char)
  (unix:sleep 3)
  (add-resume *sm*)
  (send *insp* :publish-structure)
  )

;; copy from roseus_smach/sample/state-machine-ros-sample.l
(defun smach-exec (insp &key (reset t))
  (let ((sm (send insp :state-machine))
	(mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3)))))
    (unix:sleep 1) ;; for advertize topics successfully
    (when reset
      (send sm :reset-state))
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      (unix:sleep 1)
;;	  (read-char)
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

(defun exec ()
  (ros::spin-once)
  (send *sm* :execute nil :step -1)
  (send *insp* :publish-structure)
  )

(defun demo ()
  (change-smach)
  (smach-exec *insp*))

(init)