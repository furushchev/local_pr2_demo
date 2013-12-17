(ros::roseus "simple_metric_plan")

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(setq *domain* (instance pddl-domain :init :name 'simple_metric))
(send *domain* :requirements '(:typing :action-costs))
(send *domain* :types '(item))
(send *domain* :predicates
      '(
        (PRED1 ?OBJ - item)
        (PRED2 ?OBJ - item)
        (PRED3 ?OBJ - item)
        (PRED4 ?OBJ - item)
        (PRED5 ?OBJ - item)
        (PRED6 ?OBJ - item)
        (PRED7 ?OBJ - item)
        ))

(send *domain* :functions '((total-cost)
                            (act1)
                            (act2)
                            (act3)
                            (act4)
                            (act5)
                            (act6)
                            (act7)
                            ))

(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "act1"
                 :parameters '((?OBJ item))
                 :precondition '((PRED1 ?OBJ))
                 :effect '((NOT (PRED1 ?OBJ))
                           (PRED2 ?OBJ)
                           (increase (total-cost) (act1))
                           ))
       (instance pddl-action :init
                 :name "act2"
                 :parameters '((?OBJ item))
                 :precondition '((PRED2 ?OBJ))
                 :effect '((NOT (PRED2 ?OBJ))
                           (PRED3 ?OBJ)
                           (increase (total-cost) (act2))
                           ))
       (instance pddl-action :init
                 :name "act3"
                 :parameters '((?OBJ item))
                 :precondition '((PRED1 ?OBJ))
                 :effect '((NOT (PRED1 ?OBJ))
                           (PRED3 ?OBJ)
                           (increase (total-cost) (act3))
                           ))
       (instance pddl-action :init
                 :name "act4"
                 :parameters '((?OBJ item))
                 :precondition '((PRED2 ?OBJ))
                 :effect '((NOT (PRED2 ?OBJ))
                           (PRED4 ?OBJ)
                           (increase (total-cost) (act4))
                           ))
       (instance pddl-action :init
                 :name "act5"
                 :parameters '((?OBJ item))
                 :precondition '((PRED2 ?OBJ))
                 :effect '((NOT (PRED2 ?OBJ))
                           (PRED5 ?OBJ)
                           (increase (total-cost) (act5))
                           ))
       (instance pddl-action :init
                 :name "act6"
                 :parameters '((?OBJ item))
                 :precondition '((PRED5 ?OBJ))
                 :effect '((NOT (PRED5 ?OBJ))
                           (PRED6 ?OBJ)
                           (increase (total-cost) (act6))
                           ))
       (instance pddl-action :init
                 :name "act7"
                 :parameters '((?OBJ item))
                 :precondition '((PRED6 ?OBJ))
                 :effect '((PRED2 ?OBJ)
                           (increase (total-cost) (act7))
                           ))

       ))

(dolist (ac *actlist*)
  (send *domain* :add :action ac))

(defmacro defproblem-maker (fname name domain &key objects initial-condition goal-condition metric)
  `(defun ,fname (&key objects initial-condition goal-condition metric)
    (let ((p (instance pddl-problem :init
                       :name ,name
                       :domain ,domain
                       )))
      (send p :objects (append objects ,objects))
      (send p :initial-condition (append initial-condition ,initial-condition))
      (send p :goal-condition (append goal-condition ,goal-condition))
      (send p :metric (append metric ,metric))
      p))
  )

(defproblem-maker make-problem 'simple_metric_plan 'simple_metric
                                  :objects '((OBJ1 . item))
                                  :initial-condition '((= (total-cost) 0)
                                                       (= (act1) 1)
                                                       (= (act2) 1)
                                                       (= (act3) 3)
                                                       (= (act4) 1)
                                                       (= (act5) 1)
                                                       (= (act6) 1)
                                                       (= (act7) 1)
                                                       )
                                  :metric '(minimize (total-cost))))

(setq *problem* (make-problem :initial-condition '((PRED1 OBJ1))
                                   :goal-condition '((PRED3 OBJ1))))

(setq *problem-sub* (make-problem :initial-condition '((PRED1 OBJ1))
                                  :goal-condition '((PRED4 OBJ1))))

(setq *result* (solve-pddl-planning *domain* *problem* :debug t))
(setq *result-sub* (solve-pddl-planning *domain* *problem-sub* :debug t))
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
(setq *graph* (add-failed-nodes *result-sub* (list 'act1)
                                :graph *graph*))
(send (make-readable-graph *graph*) :write-to-pdf "test.pdf")
(piped-fork "gnome-open test.pdf")

#|
(setq *problem-interrupt* (make-problem :initial-condition '((PRED2 OBJ1))
                                        :goal-condition '((PRED6 OBJ1))))
(setq *result-interrupt* (solve-pddl-planning *domain* *problem-interrupt* :debug t))
(setq *graph* (add-failed-nodes *result-interrupt* (list 'act2) :graph *graph*))
(send (make-readable-graph *graph*) :write-to-pdf "test2.pdf")
(piped-fork "gnome-open test2.pdf")

(setq *problem-recovery* (make-problem :initial-condition '((PRED6 OBJ1))
                                        :goal-condition '((PRED4 OBJ1)
                                                          (PRED6 OBJ1))))
(setq *result-recovery* (solve-pddl-planning *domain* *problem-recovery* :debug t))
(setq *graph* (add-failed-nodes *result-recovery* (list 'act2) :graph *graph*))
(send (make-readable-graph *graph*) :write-to-pdf "test3.pdf")
(piped-fork "gnome-open test3.pdf")
|#
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

(setq *sm-inspect* (pddl-graph-to-smach *graph*
                                        :add-self-return-failure-path nil))
(setq *sm* (send *sm-inspect* :state-machine))

(defun ref-smach ()
  (ros::spin-once)
  (send *sm-inspect* :publish-structure)
  (send *sm-inspect* :publish-status (list (cons 'count 0) (cons 'hoge (list 1 2 3))))
  )

#|
(defclass graph-state-manager
    :slots (graph current-arc smach)
  )

(defmethod graph-state-manager
    (:init
     (gr sm)
     (setq graph gr)
     (setq smach sm))
    (:execute (data step)
     (let ((asname (send (send sm :active-state) :name))
           (
|#

(require :pr2-utils "pr2-utils.l")
(defparameter *current-state-topic* "current_state")
(defun smach-exec (insp graph)
  (let ((sm (send insp :state-machine))
        (mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3))))
        (current-node (car (last (send graph :nodes)))))
    (unless (ros::get-topic-publisher *current-state-topic*)
      (ros::advertise *current-state-topic* std_msgs::String 5)
      (unix:sleep 1))
    (unix:sleep 1) ;; for advertize topics successfully
    (send sm :reset-state)
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      (unix:sleep 1)
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1)
      ;; also updates graph
      (let ((pair (assoc (send (send sm :active-state) :name) (send current-node :neighbor-action-alist))))
        (if pair
            (setq current-node (cdr pair))
            (ros::publish *current-state-topic*
                          (instance std::msgs::String :init
                                    :data (string-comp "\n" (send current-node :name))))
            )
      ))
    (send sm :active-state) ;; return reached goal state
    ))

