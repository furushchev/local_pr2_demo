;;(load "attention-observation.l")
;;(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; database
(defun geth (k &key (h *hash*))
  (gethash k h))

(defun seth (k v &key (h *hash*))
  (unless (boundp '*hash*) (defparameter *hash* (make-hash-table)))
  (setf (gethash k h) v))

;; conversion
(defun ros->eus (msg)
  (let ((ros-av-lst (send msg :data))
        avs tms)
    (dolist (ros-av ros-av-lst)
      (push (send ros-av :data) avs)
      (push (send ros-av :dura) tms))
    (list (nreverse avs) (nreverse tms))))

(defun eus->ros (avs-tm-lst)
  (let ((msg (instance jsk_demo_common::AngleVectorSequence :init))
        ros-av-lst)
    (dotimes (i (length (elt avs-tm-lst 0)))
      (push (instance jsk_demo_common::AngleVector :init
                      :data (elt (elt avs-tm-lst 0) i)
                      :dura (elt (elt avs-tm-lst 1) i))
            ros-av-lst))
    (send msg :data (nreverse ros-av-lst))
    msg))

;; callback
(defun update-avs (msg)
  (let ((id-str (send msg :id))
        (avs-tm (ros->eus (send msg :seq))))
    (ros::ros-info "updated: ~A" id-str)
    (seth id-str avs-tm)))

(defun return-avs (req)
  (let ((res (send req :response)))
    (send res :id (send req :id))
    (send res :data (eus->ros (geth (send req :id))))
    (ros::ros-info "sent: ~A" (send req :id))
    res))

;; main
(defun main ()
  (ros::roseus "angle_vector_manager")
  (ros::roseus-add-msgs "jsk_demo_common")
  (ros::roseus-add-srvs "jsk_demo_common")
  (ros::rate 100)
  (ros::subscribe "/angle_vector_sequence" jsk_demo_common::AngleVectorSequenceWithID #'update-avs)
  (ros::advertise-service "/angle_vector_seqence" jsk_demo_common::IdAngleVectorWithID #'return-avs)
  (do-until-key
      (while (ros::ok)
        (ros::spin-once)
        (ros::sleep)))
  (ros::exit))

;;(main)

(defmacro defaction (name arg-lst &rest form)
  `(defun ,name ,arg-lst
     (let ((function-name ,name)
           (angle-vector-number -1))
       ,@form)))

(defun send-ri-angle-vector-sequence (avs tms)
  (if (not (boundp 'function-name))
      (ros::ros-error "this function must be used in function defined by defaction"))
  (let* ((id (format nil "~A-~A" function-name (inc angle-vector-number))) ;; angle-vector-number++
         (req (instance jsk_demo_common::IdAngleVectorSequenceWithID :init
                        :id id))
         (res (ros::service-call "/angle_vector_sequence" req)))  ;; fetch avs from server
    (if res ;; TODO: must check if return nil when comes empty msg
        (send *ri* :angle-vector-sequence (elt res 0) (elt res 1))
        (progn ;; else
          (unless (ros::get-topic-publisher "/angle_vector_sequence")
            (ros::advertise "/angle_vector_sequence" jsk_demo_common::AngleVectorSequenceWithID)
            (unix:sleep 1))
          (ros::publish "/angle_vector_sequence" ;; register avs to server
                        (eus->ros (list avs tms)))
          (send *ri* :angle-vector-sequence avs tms)))))

(defun wait-interpolation ()
  (if (not (boundp 'function-name))
      (ros::ros-error "this function must be used in function defined by defaction"))
  (unwind-protect ;; TODO: check if exec follow let* when interrupted
       (send *ri* :wait-interpolation)
    (let* ((req (instance jsk_demo_common::IdAngleVectorSequenceWithID :init
                          :id (format nil "~A-~A" function-name angle-vector-number)))
           (res (ros::service-call "/angle_vector_sequence_configure" req))
           (avs-tm-lst (ros->eus (send res :data))))
      (ros::ros-info "~A ~A interrupted!" function-name angle-vector-number)
      (send *ri* :stop-motion)
      (when res ;; recovery TODO: must check if return nil when comes empty msg
        (ros::ros-info "sent recovery trajectory")
        (send *ri* :angle-vector-sequence (elt avs-tm-lst 0) (elt avs-tm-lst 1)))
      )))

(defun send-ri-angle-vector (av &optional (time 3000))
  (angle-vector-sequence (list av) (list time)))



#|
(defun angle-vector-attention (id av &optional (time 3000))
  (let (attention-barrier-return)
    (setq attention-barrier-return
          (catch :attention-barrier
            (let ((attention-action-return
                   (progn
                     (send *ri* :angle-vector av time))))
                 (unless attention-action-return
                   (throw :attention-barrier id))
                 :success
                 )))
       (warn ";; exec ~A finish with ~A~%" id attention-barrier-return)
       (cond
        ((stringp attention-barrier-return)
         (let ((str (send *ri* :get-cancel-flag)))
           (setq attention-barrier-return
                 (intern (string-upcase attention-barrier-return) *keyword-package*))
           ))
        ((null attention-barrier-return)
         (warn ";; attention-barrier-return = nil~%")
         (setq attention-barrier-return nil)) ;; just fail action
        ((eq t attention-barrier-return) ;; normal cancel
         (setq attention-barrier-return id))
        )
       (send *ri* :clear-cancel-flag)
       attention-barrier-return))
|#
