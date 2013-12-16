#!/usr/bin/env roseus

;;(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-downward-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(load "package://jsk_demo_common/euslisp/attention-observation.l")

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
      '((CLOSED)
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
        (= (COST-MOVE-TO) 100)  ;; 低いと無駄な場所に行くようになる
        (= (COST-OPEN-DOOR) 1000)  ;; 低いと冷蔵庫を一時物置き場として使うようになる
        (= (COST-PASS-TO-HUMAN) 100)  ;; 低いと人に一時ものを持ってもらって、その間に違うことをするようになる
        ))

(send *problem* :metric '(minimize (TOTAL-COST)))
(setq *domain* (instance pddl-domain :init :name 'fridgedemo))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item spot base arm))
(send *domain* :predicates '((AT ?SPOT - spot)
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
                            (COST-MOVE-TO)
                            (COST-OPEN-DOOR)
                            (COST-PASS-TO-HUMAN)))
