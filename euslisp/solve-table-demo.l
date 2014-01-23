#!/usr/bin/env roseus

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'carry_tray :domain 'traydemo))

;;使用する変数の宣言
(send *problem* :objects
      '((START . spot)
        (TABLE . spot)
        (SINK . spot)
;;        ( . spot)
        (SOMEWHERE . spot)
        (TRAY . item)
;;        (L-ARM . arm)
;;        (R-ARM . arm)
        (BOTH-ARM . arm)
        ))

;;初期条件
(send *problem* :initial-condition
      '((ON TRAY TABLE)
        (AT START)
        ))

;;終了条件
(send *problem* :goal-condition
      '((ON TRAY SINK)
        (AT START)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'traydemo))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item spot arm))
(send *domain* :predicates '((GRASP ?OBJ - item ?ARM - arm)
                             (AT ?SPOT - spot)
                             (ON ?OBJ - item ?SPOT - spot)
                             ))
(send *domain* :constants '(FLOOR - spot))


;;making action
(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?SPOT spot))
                 :precondition '((NOT (AT ?SPOT))
                                 (forall (?OBJ - item)
                                  (forall (?ARM - arm)
                                   (NOT (GRASP ?OBJ ?ARM)))))
                 :effect '((forall (?AT - spot)
                            (NOT (AT ?AT)))
                           (AT ?SPOT)))

       (instance pddl-action :init
                 :name "carry-to"
                 :parameters '((?SPOT spot))
                 :precondition '((NOT (AT ?SPOT))
                                 (exists (?OBJ - item)
                                  (forall (?ARM - arm)
                                   (GRASP ?OBJ ?ARM))))
                 :effect '((forall (?AT - spot)
                            (NOT (AT ?AT)))
                           (AT ?SPOT)))


       (instance pddl-action :init
                 :name "carry-to_f"
                 :parameters '((?SPOT spot))
                 :precondition '((NOT (AT ?SPOT))
                                 (exists (?OBJ - item)
                                  (forall (?ARM - arm)
                                   (GRASP ?OBJ ?ARM))))
                 :effect '((forall (?AT - spot)
                            (NOT (AT ?AT)))
                           (AT FLOOR)
                           (forall (?OBJ - item)
                            (forall (?ARM - arm)
                             (when (GRASP ?OBJ ?ARM)
                               (AND (NOT (GRASP ?OBJ ?ARM))
                               (ON ?OBJ FLOOR)))))))

       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((exists (?AT - spot)
                                  (AND (AT ?AT) (ON ?OBJ ?AT)))
                                 (forall (?ITEM - item)
                                  (NOT (GRASP ?ITEM ?ARM))))
                 :effect '((GRASP ?OBJ ?ARM)
                           (forall (?AT - spot)
                            (NOT (ON ?OBJ ?AT)))))

       (instance pddl-action :init
                 :name "place"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((GRASP ?OBJ ?ARM))
                 :effect '((NOT (GRASP ?OBJ ?ARM))
                           (forall (?AT - spot)
                            (when (AT ?AT)
                              (ON ?OBJ ?AT)))))
       ))

;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))

(setq *graph* (pddl-plan-to-graph nil :failed-nodes (list 'carry-to) :readable t :debug t))
(pprint *result*)

;; to smach
(load "package://roseus_smach/src/pddl2smach.l")
(load "package://roseus_smach/src/state-machine-ros.l")

(setq *sm-inspect* (pddl-graph-to-smach *graph*))
(setq *sm* (send *sm-inspect* :state-machine))
(send *sm* :start-state "(move-to table)")
(send *sm-inspect* :publish-all-status)
(send *sm-inspect* :publish-structure)
