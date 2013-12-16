
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(if (not (boundp '*scene*)) (setq *scene* (make-eng2-scene)))

(defun string-comp (ss s)
  "sに対するssの補集合"
  (if (string= ss s)
      ""
      (if (> (length ss) (length s))
          s
          (if (string= ss s :end2 (length ss))
              (string-comp ss (subseq s (length ss)))
              (concatenate string (subseq s 0 1)
                           (string-comp ss (subseq s 1)))))))

(defun make-73b2-move-to-cost (opname &key (scene *scene*) (multiply 1.0))
  "73b2のspotに距離に応じたcostを自動生成する"
  (let ((73b2-spots (remove-if-not #'(lambda (s) (substringp "/eng2/7f/room73B2-" (send s :name))) (send scene :spots)))
        ret-exp)
    (dolist (spot-pair (combination 73b2-spots 2))
      (push `(= ,(list opname
                       (string-upcase
                        (string-comp "/eng2/7f/room73B2-"
                                     (send (car spot-pair) :name)))
                       (string-upcase
                        (string-comp "/eng2/7f/room73B2-"
                                     (send (cadr spot-pair) :name)))
                       )
                ,(floor (* multiply
                           (norm
                            (send (car spot-pair) :difference-position
                                  (cadr spot-pair)))))
                )
            ret-exp))
    ret-exp))

(provide :solve-73b2-demo-util)
