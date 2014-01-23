#!/usr/bin/env roseus

;;(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(load "package://jsk_demo_common/euslisp/attention-observation.l")
(require :solve-73b2-demo-util "solve-73b2-demo-util.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'bring_can :domain 'fridgedemo))

;;使用する変数の宣言
(send *problem* :objects
      '((START . spot)
        (FRIDGE . base)
        (TABLEFRONT . spot)
        (TABLESIDE . spot)
        (TABLE . base)
        (TRASHBOXFRONT . spot)
        (TRASHBOX . base)
        (SINKFRONT . spot)
        (COUNTER-SIDE . spot)
        (SINK . base)
        (IHFRONT . spot)
        (IH . base)
        (IHBUTTON . item)
        (CAN . item)
        (CUP . item)
        (DUST . item)
        (TRAY . item)
        (L-ARM . arm)
        (R-ARM . arm)
        ;; (ARMS . arm)
        ))

;;初期条件
(send *problem* :initial-condition
      `((CLOSED)
        (PET-CLOSED)
        (AT START)
        (REACHABLE FRIDGE FRIDGEFRONT)
        (REACHABLE TRASHBOX TRASHBOXFRONT)
        (REACHABLE SINK SINKFRONT)
        (REACHABLE IH IHFRONT)
        (REACHABLE TABLE TABLESIDE)
        (REACHABLE TABLE TABLEFRONT)
        (REACHABLE HUMAN HUMANFRONT)
        (TYPICALLY-AT TRAY TABLE)
        (TYPICALLY-AT TRAY SINK)
        (TYPICALLY-AT CAN FRIDGE)
        (TYPICALLY-AT CAN HUMAN)
        (TYPICALLY-AT IHBUTTON IH)
        (TYPICALLY-AT KETTLE IH)
        (TYPICALLY-AT PET FRIDGE)
        (TYPICALLY-AT CUP TABLE)
        (TYPICALLY-AT CUP HUMAN)
        (TYPICALLY-AT DUST TABLE)
        (TYPICALLY-AT DUST TRASHBOX)
        (PLACABLE COUNTER-SIDE)
        (PLACABLE TABLESIDE)
        (PLACABLE TABLEFRONT)
        (PLACABLE SINKFRONT)
        (PLACABLE IHFRONT)
        (ON TRAY TABLE)
        (ON DUST TABLE)
        (ON PET FRIDGE)
        (ON CAN FRIDGE)
        (POURABLE CUP)
        (ON CUP SINK)
        (ON IHBUTTON IH)
        (ON KETTLE IH)
        (= (TOTAL-COST) 0)
        (= (COST-OTHER) 1)
;;        (= (COST-MOVE-TO) 100)  ;; 低いと無駄な場所に行くようになる
        (= (COST-OPEN-DOOR) 1000)  ;; 低いと冷蔵庫を一時物置き場として使うようになる
        (= (COST-PASS-TO-HUMAN) 100)  ;; 低いと人に一時ものを持ってもらって、その間に違うことをするようになる
        ,(make-73b2-move-to-cost 'COST-MOVE-TO)
        ))

;;終了条件
(send *problem* :goal-condition
      '((CLOSED)
        (PET-CLOSED)
        (AT START)
        (ON TRAY SINK)
        ;; (ON DUST TRASHBOX)
        ;; (ON PET FRIDGE)
        ;; (NOT (POURABLE CUP))
        ;; (ON CAN HUMAN)
        ;; (ON CUP HUMAN)
        ))

#| ;; if pet
(send *problem* :goal-condition
      '((CLOSED)
        (PET-CLOSED)
        (AT START)
        (ON TRAY SINK)
        (ON DUST TRASHBOX)
        (ON PET FRIDGE)
        (NOT (POURABLE CUP))
        (ON CAN HUMAN)
        (ON CUP HUMAN)
        ))
|#

;; metric
(send *problem* :metric '(minimize (TOTAL-COST)))

(setq *problem-fridge* (copy-object *problem*))
(send *problem-fridge* :goal-condition
      '((CLOSED)
        (PET-CLOSED)
        (AT START)
        (ON CAN HUMAN)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'fridgedemo))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item spot base arm))
(send *domain* :predicates '((AT ?SPOT - spot)
                             (PLACABLE ?AT - spot)
                             (ON ?OBJ - item ?BASE - base)
                             (REACHABLE ?OBJ - base ?SPOT - spot)
                             (CLOSED)
                             (PET-CLOSED)
                             (ARM-GRASP ?OBJ - item ?ARM - arm)
                             (TYPICALLY-AT ?OBJ - item ?BASE - base)
                             (POURABLE ?OBJ - item)
                             (BUTTON-PRESSED ?OBJ - item)
                             (BOILED ?OBJ - item)
                             ))

;; set constants
(send *domain* :constants '(PET - item
                            KETTLE - item
                            FRIDGEFRONT - spot
                            HUMAN - base
                            HUMANFRONT - spot
                            ARMS - arm))

;; set functions
(send *domain* :functions '((TOTAL-COST)
                            (COST-OTHER)
                            (COST-MOVE-TO spot1 - spot spot2 - spot)
                            (COST-OPEN-DOOR)
                            (COST-PASS-TO-HUMAN)))

;;making action
(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "pass-to-human"
                 :parameters '((?OBJ item))
                 :precondition '((AT HUMANFRONT)
                                 (TYPICALLY-AT ?OBJ HUMAN)
                                 (exists (?ARM - arm)
                                  (ARM-GRASP ?OBJ ?ARM)))
                 :effect '((forall (?ARM - arm)
                            (when (ARM-GRASP ?OBJ ?ARM)
                              (NOT (ARM-GRASP ?OBJ ?ARM))))
                           (ON ?OBJ HUMAN)
                           (increase (TOTAL-COST) (COST-PASS-TO-HUMAN))
                           )
                 )
#|
       (instance pddl-action :iniit
                 :name "leave-to"
                 :parameters '((?TO spot))
                 :precondition '((NOT (AT ?TO))
                                 (exists (?ARM - arm)
                                  (exists (?OBJ - item)
                                   (ARM-GRASP ?OBJ ?ARM)))
                                 (NOT (REACHING ?TO))
                                 (CLOSED))
                 :effect '((exists (?NEXT - spot)
                            (exists (?AT - spot)
                             (when (AND (AT ?AT)
                                        (OR (NEXT-TO ?AT ?NEXT)
                                            (NEXT-TO ?NEXT ?AT)))
                               (NOT (AT ?AT))
                               (AT ?NEXT))))
                           (AT MOVING)
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "carry-around"
                 :parameters '((?SPOT - spot))
                 :precondition '((AT MOVING))
                 :effect '((forall (?SPOT - spot)
                            (NOT (AT ?SPOT)))
                           (exists (?OBJ - item)
                            (exists (?TO - spot)
                             (when (CARRYING ?OBJ ?TO)
                               (AT ?TO)))))
                 )

       (instance pddl-action :init
                 :name "reach-to"
                 :parameters
|#
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?TO spot))
                 :precondition '((NOT (AT ?TO))
                                 (CLOSED))
                 :effect '((forall (?AT - spot)
                            (when (AT ?AT)
                              (increase (TOTAL-COST) (COST-MOVE-TO ?AT ?TO))))
                           (AT ?TO)
                           (forall (?FROM - spot)
                            (when (NOT (= ?TO ?FROM))
                              (NOT (AT ?FROM))))
                           )
                 )

       (instance pddl-action :init
                 :name "open-door"
                 :parameters '()
                 :precondition '((AT FRIDGEFRONT)
                                 (exists (?ARM - arm)
                                  (forall (?OBJ - item)
                                   (NOT (ARM-GRASP ?OBJ ?ARM))))
                                 (CLOSED))
                 :effect '((NOT (CLOSED))
                           (increase (TOTAL-COST) (COST-OPEN-DOOR))
                           ))

       (instance pddl-action :init
                 :name "close-door"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT FRIDGEFRONT))
                 :effect '((CLOSED)
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )
       #|
       (instance pddl-action :init
       :name "close_f"
       :parameters '()
       :precondition '((NOT (CLOSED))
       (AT PRECLOSE))
       :effect '())
       |#
       #|
       (instance pddl-action :init
       :name "try-close"
       :parameters '()
       :precondition '((NOT (CLOSED))
       (AT SOMEWHERE))
       :effect '((CLOSED)))
       |#
       #|
       (instance pddl-action :init
       :name "close_try_f"
       :parameters '()
       :precondition '((NOT (CLOSED))
       (AT SOMEWHERE))
       :effect '())
       |#

       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((exists (?SPOT - spot)
                                  (exists (?BASE - base)
                                   (AND (AT ?SPOT)
                                        (REACHABLE ?BASE ?SPOT)
                                        (ON ?OBJ ?BASE)
                                        (NOT (AND (= ?SPOT FRIDGEFRONT)
                                                  (CLOSED)))
                                        )))
                                 (NOT (exists (?OBJ2 - item)
                                       (OR (ARM-GRASP ?OBJ2 ?ARM)
                                           (ARM-GRASP ?OBJ2 ARMS)))))
                 :effect '((ARM-GRASP ?OBJ ?ARM)
                           (forall (?BASE - base)
                            (NOT (ON ?OBJ ?BASE)))
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "place"
                 :parameters '((?BASE base) (?ARM arm))
                 :precondition '((exists (?SPOT - spot)
                                  (AND (AT ?SPOT)
                                   (REACHABLE ?BASE ?SPOT)))
                                 (exists (?OBJ - item)
                                  (AND (ARM-GRASP ?OBJ ?ARM)
                                   (TYPICALLY-AT ?OBJ ?BASE))))
                 :effect '((forall (?OBJ - item)
                            (when (ARM-GRASP ?OBJ ?ARM)
                              (AND (NOT (ARM-GRASP ?OBJ ?ARM))
                                   (ON ?OBJ ?BASE))))
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "pour-into"
                 :parameters '((?DRINK item) (?OBJ item))
                 :precondition '((POURABLE ?OBJ)
                                 (exists (?ARM - arm)
                                  (ARM-GRASP ?OBJ ?ARM))
                                 (exists (?ARM - arm)
                                  (ARM-GRASP ?DRINK ?ARM))
                                 (NOT (PET-CLOSED)))
                 :effect '((NOT (POURABLE ?OBJ))
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "open-pet"
                 :parameters '()
                 :precondition '((PET-CLOSED)
                                 (exists (?ARM - arm)
                                  (ARM-GRASP PET ?ARM)))
                 :effect '((NOT (PET-CLOSED))
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "close-pet"
                 :parameters '()
                 :precondition '((NOT (PET-CLOSED))
                                 (exists (?ARM - arm)
                                  (ARM-GRASP PET ?ARM)))
                 :effect '((PET-CLOSED)
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       ;; task yakan
       (instance pddl-action :init
                 :name "turn-on"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((forall (?OBJ - item)
                                  (and
                                   (not (ARM-GRASP ?OBJ ?ARM))
                                   (not (ARM-GRASP ?OBJ ARMS))))
                                 (exists (?SPOT - spot)
                                  (exists (?BASE - base)
                                   (and (AT ?SPOT)
                                        (REACHABLE ?BASE ?SPOT)
                                        (ON ?OBJ ?BASE)
                                        (not (BUTTON-PRESSED ?OBJ))
                                        ))))
                 :effect '((BUTTON-PRESSED ?OBJ)
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       (instance pddl-action :init
                 :name "turn-off"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((forall (?OBJ - item)
                                  (and
                                   (not (ARM-GRASP ?OBJ ?ARM))
                                   (not (ARM-GRASP ?OBJ ARMS))))
                                 (exists (?SPOT - spot)
                                  (exists (?BASE - base)
                                   (and (AT ?SPOT)
                                        (REACHABLE ?BASE ?SPOT)
                                        (ON ?OBJ ?BASE)
                                        (BUTTON-PRESSED ?OBJ)
                                        ))))
                 :effect '((not (BUTTON-PRESSED ?OBJ))
                           (when (exists (?BASE - base)
                                         (ON ?OBJ ?BASE)
                                         (ON KETTLE ?BASE))
                             (BOILED KETTLE))
                           (increase (TOTAL-COST) (COST-OTHER)))
                 )

       ))
;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))

;;
;; solve planning
;;

(setq *result-tray* (solve-pddl-planning *domain* *problem* :optimize-state nil))

(setq *result-fridge* (solve-pddl-planning *domain* *problem-fridge* :optimize-state nil))

(setq *graph* (add-failed-nodes *result-fridge* (list 'move-to)
                                :graph (make-graph-from-pddl-results (list *result-tray*) :node-name :pprint)))
(make-readable-graph *graph* :copy nil)

;; execute state-machine and publish every step
(defun smach-exec (smip)
  (let ((mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3)))))
    (unix:sleep 1) ;; for advertize topics successfully
                                        ;    (send sm :reset-state)
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      (unix:sleep 1)
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

(ros::set-param "~display_graph" nil)

(when (and (ros::has-param "~display_graph")
           (ros::get-param "~display_graph"))
  (piped-fork "gnome-open debug.pdf")
  )

;; to smach
(load "package://roseus_smach/src/pddl2smach.l")
(load "package://roseus_smach/src/state-machine-ros.l")


(setq *sm-data* (list (cons 'count 0) (cons 'hoge (list 1 2 3))))
(setq *sm-inspect* (pddl-graph-to-smach *graph*))

(setq *sm* (send *sm-inspect* :state-machine))



;;  (send *sm-inspect* :reset-state)
;;  (exec-smach-with-interrupt *sm-inspect*)
                                        ;  (smach-exec *sm-inspect*)

;;(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
;;  (ros::exit))
