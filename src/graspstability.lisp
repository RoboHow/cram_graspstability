(in-package :cram-graspstability)

(define-condition grasp-stability-violated () ())

(defvar *control-service* "/grasp_stability_estimator/control")
(defvar *state-topic* "/grasp_stability_estimator/state")

(defun config (&key (control-service nil control-service-p)
                    (state-topic nil state-topic-p))
  (when control-service-p
    (setf *control-service* control-service))
  (when state-topic-p
    (setf *state-topic* state-topic)))

(defmacro with-grasp-stability-awareness (context-id
                                          worst-quality
                                          &body body)
  `(let ((state-fluent (make-fluent :name "state-fluent"
                                    :value :stopped))
         (return-value nil))
     (flet ((stability-state-callback (msg)
              (with-fields (measurement_context_id
                            grasp_quality) msg
                (cond
                  ((and
                    (string= measurement_context_id ,context-id)
                    (or (< grasp_quality ,worst-quality)))
                   (cpl:setf (value state-fluent) :failed))
                  (t (format t "Reported grasp satisfies our requirements.~%"))))))
       (let* ((control-service "/grasp_stability_estimator/control")
              (state-topic "/grasp_stability_estimator/state")
              (state-subscriber (roslisp:subscribe
                                 state-topic
                                 "grasp_stability_msgs/GraspStability"
                                 #'stability-state-callback)))
         (unwind-protect
              (progn
                (roslisp:call-service
                 control-service
                 "grasp_stability_msgs/Control"
                 :command (roslisp-msg-protocol:symbol-code
                           'grasp_stability_msgs-srv:control-request
                           :ctrl_start)
                 :measurement_context_id ,context-id)
                (cpl:setf (value state-fluent) :running)
                (pursue
                  (block state-fluent-checker
                    (loop do
                      (wait-for state-fluent)
                      (case (value state-fluent)
                        (:failed (return-from state-fluent-checker nil))
                        (:finished (return-from state-fluent-checker t)))))
                  (seq
                    (block code-executer
                      (setf return-value (progn ,@body))
                      (cpl:setf (value state-fluent) :finished)
                      return-value))))
           (roslisp:unsubscribe state-subscriber)
           (roslisp:call-service
            control-service
            "grasp_stability_msgs/Control"
            :command (roslisp-msg-protocol:symbol-code
                      'grasp_stability_msgs-srv:control-request
                      :ctrl_stop)
            :measurement_context_id ,context-id)
           (when (eql (value state-fluent) :failed)
             (error 'grasp-stability-violated))
           return-value)))))
