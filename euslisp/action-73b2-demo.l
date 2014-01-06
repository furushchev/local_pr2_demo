#!/usr/bin/env roseus


(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "package://jsk_2013_10_pr2_73b2/euslisp/setup.l")
;; (load "package://jsk_demo_common/euslisp/attention-observation.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")

(defun pick-tray-manual (&key (torso 84.2881))
  (pr2-pick-tray-pose :reset-pose t)
  (send *pr2* :torso :waist-z :joint-angle (- torso 34.07))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :arms :wait t)
  (unix:sleep 2)
  (speak-jp "trayがみつかりました")
;;  (read-char)
  (unix:sleep 2)
  (send *ri* :angle-vector (float-vector torso 23.8014 -13.2479 96.7415 -38.8001 57.1561 -89.7842 163.472 -20.7195 -13.2837 -96.546 -37.4096 -58.0199 -90.2788 197.008 1.13872 44.6386) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (float-vector (- torso 2) 25.2541 -4.7791 98.5807 -42.9741 61.1682 -81.0683 164.169 -22.3653 -4.8541 -98.3104 -41.8459 -61.8961 -81.4948 196.436 1.14264 44.5448) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :arms :gain 1)
  (send *ri* :angle-vector (float-vector torso 23.8014 -13.2479 96.7415 -38.8001 57.1561 -89.7842 163.472 -20.7195 -13.2837 -96.546 -37.4096 -58.0199 -90.2788 197.008 1.13872 44.6386))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :arms :move-end-pos #f(0 0 -250))
  (send *pr2* :look-at-hand :arms)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  t)

(defun pick-tray-table (&rest args)
  (let ((torso 84.2881))
    (if (> (length args) 0)
        (let ((fail-state (cdr (assoc 'fail-state args))))
          (when (equal fail-state :temp)
            (setq torso 220))))
  (format t "(pick-tray)~%")
  (pick-tray)
;;  (pick-tray-manual :torso torso)
  )
  t)

(defun place-tray-manual (&key (torso 84.2881))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :torso :waist-z :joint-angle torso)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix:sleep 2)
;;  (read-char)
  (send *pr2* :arms :move-end-pos #f(0 0 250))
  (send *pr2* :look-at-hand :arms)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (float-vector (- torso 2) 25.2541 -4.7791 98.5807 -42.9741 61.1682 -81.0683 164.169 -22.3653 -4.8541 -98.3104 -41.8459 -61.8961 -81.4948 196.436 1.14264 44.5448) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :arms :wait t)
  (send *ri* :angle-vector (float-vector torso 23.8014 -13.2479 96.7415 -38.8001 57.1561 -89.7842 163.472 -20.7195 -13.2837 -96.546 -37.4096 -58.0199 -90.2788 197.008 1.13872 44.6386) 5000)
  (send *ri* :wait-interpolation)
  (send *pr2* :arms :move-end-pos #f(0 0 -250))
  (send *pr2* :look-at-hand :arms)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (pr2-reset-pose)
  t)


(defun place-tray-table (&rest args)
  (format t "(place-tray)~%")
  (place-tray :table-height 820
              :go-forward 200)
;;  (place-tray-manual :torso 220)
  t)

(defun move-to-sink-front ()
  (format t "(move-to-sink-front)~%")
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :torso :waist-z :joint-angle 200)
  (let* ((res (move-to-spot "/eng2/7f/room73B2-sink-front1"
                            :angle-vector (send *pr2* :angle-vector)
                            :tilt-enable nil
                            :clear-costmap t))
         (cpos (send *ri* :state :worldcoords))
         (table-side-pos (send *scene* :spot "/eng2/7f/room73B2-table-side1"))
         (sink-pos (send *scene* :spot "/eng2/7f/room73B2-sink-front0"))
         (diff-sink (norm (send cpos :difference-position sink-pos)))
         (diff-tray (norm (send cpos :difference-position table-side-pos))))
    (format t "res: ~A~%current pos: ~A~%table-side: ~A~%sinkfront: ~A~%diff-sink~A~%diff-tray~A~%"
            res cpos table-side-pos sink-pos diff-sink diff-tray)
    (cond
     ((eq res t) t)
     ((and (not res) (< diff-tray 2000)) :start)
     ((and (not res) (< diff-sink 2000)) :goal)
     ((not res) :temp)
     (t t))
    ))

(defun move-to (to &rest arg)
  (format t "(move-to ~A)~%" (symbol-name to))
  (cond
   ((string= "SINKFRONT" (symbol-name to))
    (move-to-sink-front))
   ((string= "TABLE-SIDE" (symbol-name to))
    (progn (move-to-spot "/eng2/7f/room73B2-table-side1"
                         :angle-vector (pr2-pick-tray-pose)
                         :clear-costmap t
                         :tilt-enable nil) t))
   ((string= "FRIDGEFRONT" (symbol-name to))
    (progn
      (change-inflation-range 0.7)
      (move-to-spot "/eng2/7f/room73B2-fridge-front"
                         :clear-costmap nil
                         :tilt-enable nil)
      (change-inflation-range 0.4)
      t))
   (t (return-from move-to t))
   ))

(defun revert-to-start (arg)
  (format t "(revert-to-start)~%")
  (setf (cdr (assoc 'fail-state arg)) :start)
  (move-to-spot "/eng2/7f/room73B2-table-side1"
                :clear-costmap t
                :tilt-enable nil)
  (place-tray :table-height 760
              :go-forward 200)
;;  (place-tray-manual)
  )

(defun continue-move-to (arg)
  (setf (cdr (assoc 'fail-state arg)) :goal)
  (move-to-spot "/eng2/7f/room73B2-sink-front0"
                :tilt-enable t
                :clear-costmap nil)
  (place-tray :table-height 820)
;;  (place-tray-manual :torso 200)
  )

(defun place-tray-temporally (arg)
  (format t "(place-tray-temporally)~%")
  (setf (cdr (assoc 'fail-state arg)) :temp)
  (move-to-spot "/eng2/7f/room73B2-counter-side"
                :tilt-enable nil
                :clear-costmap t)
  (place-tray :table-height 900
              :go-forward 50)
;;  (place-tray-manual :torso 220)
  )

(defun resume-tray (&rest arg)
  (format t "(resume-tray)~%")
  (let ((fail-state (cdr (assoc 'fail-state arg))))
    (format t "fail-state: ~A~%" fail-state)
    (case fail-state
      (:start :start)
      (:temp (progn
               (pr2-pick-tray-pose)
               (send *pr2* :torso :waist-z :joint-angle 150)
               (move-to-spot "/eng2/7f/room73B2-counter-side"
                             :angle-vector (send *pr2* :angle-vector)
                             :tilt-enable nil
                             :clear-costmap t)
               :temp))
      (:goal :goal)
      ;;	  (nil t)
      (t t)))
  )

(defun resume-fridge (&rest arg)
  (format t "(resume-fridge)~%")
  t)

(setq *use-arm* :larm)
(defun open-door (&rest arg)
  (format t "(open-door)~%")
  (open-fridge-door :use-arm *use-arm*)
  t)

(defun close-door (&rest arg)
  (format t "(close-door)~%")
  (close-fridge :use-arm *use-arm*)
  t)

(defun grasp-can-fridge (&rest arg)
  (format t "(grasp-can)~%")
  (setq *type* "georgia")
  (grasp-can :use-arm *use-arm*)
  t)

(defun pass-to-human (&rest arg)
  (format t "(pass-to-human)~%")
  (send *ri* :go-pos-unsafe 0 0 -90)
  (send *ri* :go-pos-unsafe 1 0)
  (hand-over *use-arm* :wait-shock t)
  t)
  
(defun start (&rest arg)
  (format t "(start)~%")
  (pr2-init)
  t)

(defun tray ()
  (move-to 'table-side)
;;  (pick-tray-manual)
  (pick-tray)
  (move-to-sink-front)
;;  (place-tray-manual :torso 220)
;;  (move-to 'fridgefront)
;;  (open-door)
;;  (grasp-can)
;;  (close-door)
;;  (pass-to-human)
  )

(defun temp ()
  (place-tray-temporally (list (cons 'fail-state nil)))
  (move-to 'fridgefront)
  (open-door)
  (grasp-can)
  (close-door)
  (pass-to-human)
;;  (resume-tray (list (cons 'fail-state :temp)))
  (pr2-pick-tray-pose)
  (send *pr2* :torso :waist-z :joint-angle 186)
  (move-to-spot "/eng2/7f/room73B2-counter-side"
                :angle-vector (send *pr2* :angle-vector)
                :tilt-enable nil
                :clear-costmap t)

;;  (pick-tray-table (list (cons 'fail-state :temp)))
  (pick-tray-manual :torso 220)
  (move-to-sink-front)
  (place-tray-manual :torso 220)
  )

(provide :action-73b2-demo)