(require :pr2eus-openrave "package://pr2eus_openrave/pr2eus-openrave.l")
(ros::load-ros-manifest "tabletop_object_detector")
(ros::load-ros-manifest "manipulation_msgs")

(defvar *tt-res* nil)

(defun template-grasp-eus (grasp-target-position &key (arm :rarm) (tt-res *tt-res*))
  (let ((gpsrv-name "/pr2_template_grasp_planner")
        (gp-req (instance manipulation_msgs::GraspPlanningRequest :init))
        counter)

    (unless tt-res
      (setq tt-res (tabletop-detect)))
    (ros::ros-info "Calling template grasp planner")
    (send gp-req :target :cluster (find-nearest-cluster (send tt-res :clusters) grasp-target-position))
    (send gp-req :target :region :roi_box_pose (send tt-res :table :pose))

    (setq gp-res (ros::service-call gpsrv-name gp-req))
    (unless (= (send gp-res :error_code :value)
               manipulation_msgs::GraspPlanningErrorCode::*SUCCESS*)
      (ros::ros-error (format nil "Template grasp planner returned error code ~A"
                              (send gp-res :error_code :value)))
      (return-from template-grasp-eus nil))

    (let ((world-z-offset (float-vector 0 0 51))
          (local-endpos-offset
           (send (send *pr2* (case arm (:rarm :r_wrist_roll_link) (t :l_wrist_roll_link)))
                 :transformation (send *pr2* arm :end-coords))))
      (setq grasp-cds-lst
            (mapcar #'(lambda (gp)
                        (let ((cds (ros::tf-pose->coords (send gp :grasp_pose :pose))))
                          (send cds :translate world-z-offset :world) ;; /base_link -> /base_footprint
                          (send cds :transform local-endpos-offset) ;; wrist_roll_link -> end-coords
                          cds)) (send gp-res :grasps))))

    (setq counter 0)
    (dolist (grasp-cds grasp-cds-lst)
      (let (orres
            (pre-grasp-cds;; offset
             (let ((c (send grasp-cds :copy-worldcoords)))
               (send c :translate (float-vector
                                   (* (send (elt (send gp-res :grasps) counter)
                                             :approach :desired_distance)
                                      1000 -2)
                                   0 0)) c)) ;; approach offset
            hand-coords-result
            hand-coords)

        (setq hand-coords-result (send *ri* :show-goal-hand-coords grasp-cds arm))
        ;; temporary
        (when (and
               ;;hand direction should be downword

               (>= (v. (send (send grasp-cds :worldcoords) :x-axis)
                       (float-vector 0 0 -1)) 0)

               ;;higher than table

               (> (elt (send grasp-cds :pos) 2)
                  (+ (* (send tt-res :table
                              :pose :pose :position :z) 1000) 30))


               (setq orres (send *ri* :move-end-coords-plan pre-grasp-cds
                                 :move-arm arm :use-torso t :send-trajectory nil))
               )
          (ros::ros-info "orres:~A" orres)

          (let (eusres)
            (send *pr2* :angle-vector (car (last (elt orres 0))))
            (setq eusres
                  (send *pr2* arm :inverse-kinematics grasp-cds
                        :rotation-axis t))

            (ros::ros-info "eusres: ~A" eusres)
            (when eusres
              (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
              (send *ri* :angle-vector-sequence (elt orres 0) (elt orres 1))
              (send *ri* :wait-interpolation)
              (send *ri* :update-robot-state)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :pre_grasp_posture :position) 0))
                    :wait t :effort 20)
              (send *ri* :angle-vector eusres 2000)
              (send *ri* :wait-interpolation)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :grasp_posture :position) 0))
                    :wait t :effort 20)
              (return t))))
        (setq counter (+ counter 1))))
    ))

(defun tabletop-detect ()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init)) res)
    (setq res (ros::service-call "tabletop_segmentation" req))
    (if (= 4 (send res :result))
      res
      nil)))

(defun find-nearest-cluster (clusters target-position)
  (let (dist-list min-id)
    (setq dist-list
          (mapcar #'(lambda(cl)
                      (distance2
                       (scale (/ 1.0 (length (send cl :points)))
                              (reduce #'v+
                              (mapcar #'(lambda(pt)
                                          (ros::tf-point->pos pt))
                                      (send cl :points))))
                       (ros::tf-point->pos target-position))
                      )
                  clusters))
    (setq min-id (position (apply #'min dist-list) dist-list))
    (elt clusters min-id)))

(defun reset-collider ()
  (call-empty-service "/collider_node/reset"))

(if (not (boundp '*pr2*)) (pr2-init))

(defun test-grasp ()
  (reset-collider)
  (warn "~A~%"
        (template-grasp-eus (instance geometry_msgs::Point :init
                                      :x 0.6 :y 0.0 :z 0.8)
                            :arm :rarm)))

(defaction action-template-grasp-eus (grasp-target-position &key (arm :rarm) (tt-res *tt-res*))
  (let ((gpsrv-name "/pr2_template_grasp_planner")
        (gp-req (instance manipulation_msgs::GraspPlanningRequest :init))
        (pr2c (send (send *pr2* :worldcoords) :copy-coords))
        counter)

    (send *pr2* :reset-coords)

    (unless tt-res
      (setq tt-res (tabletop-detect)))
    (ros::ros-info "Calling template grasp planner")
    (send gp-req :target :cluster (find-nearest-cluster (send tt-res :clusters) grasp-target-position))
    (send gp-req :target :region :roi_box_pose (send tt-res :table :pose))

    (setq gp-res (ros::service-call gpsrv-name gp-req))
    (unless (= (send gp-res :error_code :value)
               manipulation_msgs::GraspPlanningErrorCode::*SUCCESS*)
      (ros::ros-error (format nil "Template grasp planner returned error code ~A"
                              (send gp-res :error_code :value)))
      (return-from template-grasp-eus nil))

    (let ((world-z-offset (float-vector 0 0 51))
          (local-endpos-offset
           (send (send *pr2* (case arm (:rarm :r_wrist_roll_link) (t :l_wrist_roll_link)))
                 :transformation (send *pr2* arm :end-coords))))
      (setq grasp-cds-lst
            (mapcar #'(lambda (gp)
                        (let ((cds (ros::tf-pose->coords (send gp :grasp_pose :pose))))
                          (send cds :translate world-z-offset :world) ;; /base_link -> /base_footprint
                          (send cds :transform local-endpos-offset) ;; wrist_roll_link -> end-coords
                          cds)) (send gp-res :grasps))))

    (setq counter 0)
    (dolist (grasp-cds grasp-cds-lst)
      (let (orres
            (pre-grasp-cds;; offset
             (let ((c (send grasp-cds :copy-worldcoords)))
               (send c :translate (float-vector
                                   (* (send (elt (send gp-res :grasps) counter)
                                             :approach :desired_distance)
                                      1000 -2)
                                   0 0)) c)) ;; approach offset
            hand-coords-result
            hand-coords)

        (setq hand-coords-result (send *ri* :show-goal-hand-coords grasp-cds arm))
        ;; temporary
        (when (and
               ;;hand direction should be downword

               (>= (v. (send (send grasp-cds :worldcoords) :x-axis)
                       (float-vector 0 0 -1)) 0)

               ;;higher than table

               (> (elt (send grasp-cds :pos) 2)
                  (+ (* (send tt-res :table
                              :pose :pose :position :z) 1000) 30))

               (call-empty-service "/collider_node/reset")
               (setq orres (send *ri* :move-end-coords-plan pre-grasp-cds
                                 :move-arm arm :use-torso t :send-trajectory nil))
               )
          (ros::ros-info "orres:~A" orres)

          (let (eusres)
            (send *pr2* :angle-vector (car (last (elt orres 0))))
            (setq eusres
                  (send *pr2* arm :inverse-kinematics grasp-cds
                        :rotation-axis t))

            (ros::ros-info "eusres: ~A" eusres)
            (when eusres
              (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :pre_grasp_posture :position) 0))
                    :wait t :effort 20)
;;              (send *ri* :angle-vector-sequence (elt orres 0) (elt orres 1))
              (send-ri-angle-vector-sequence (append (elt orres 0) (list eusres))
                                             (append (mapcar #'(lambda (x) (* x 2)) (elt orres 1)) (list 5000)))
              (when (memq nil (send *ri* :wait-interpolation))
                (send *pr2* :transform pr2c)
                (return-from action-template-grasp-eus :interrupt))
              (send *ri* :update-robot-state)
              ;; (send *ri* :move-gripper arm
              ;;       (* 2 (elt (send (car (send gp-res :grasps))
              ;;                       :pre_grasp_posture :position) 0))
              ;;       :wait t :effort 20)
              ;; (send *ri* :angle-vector eusres 2000)
              ;; (send-ri-angle-vector eusres 5000)
              ;; (when (memq nil (send *ri* :wait-interpolation))
              ;;   (send *pr2* :transform pr2c)
              ;;   (return-from action-template-grasp-eus :interrupt))
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :grasp_posture :position) 0))
                    :wait t :effort 20)
              (return t))))
        (setq counter (+ counter 1))))
    (send *pr2* :transform pr2c)
    ))
