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
                                          worst-stability
                                          &body body)
  `(let ((state-fluent (make-fluent :name "state-fluent"
                                    :value :stopped))
         (return-value nil))
     (flet ((stability-state-callback (msg)
              (with-fields (context_id
                            stability) msg
                (cond
                  ((and
                    (string= context_id ,context-id)
                    (< stability ,worst-quality))
                   (cpl:setf (value state-fluent) :failed))
                  (t (format
                      t "Reported grasp satisfies our requirements.~%"))))))
       (let* ((state-subscriber (roslisp:subscribe
                                 *state-topic*
                                 "robohow_common_msgs/GraspStability"
                                 #'stability-state-callback)))
         (unwind-protect
              (progn
                (roslisp:call-service
                 control-service
                 "robohow_common_msgs/Control"
                 :command (roslisp-msg-protocol:symbol-code
                           'robohow_common_msgs-srv:graspstabilitycontrol
                           :start)
                 :context_id ,context-id)
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
            *control-service*
            "robohow_common_msgs/Control"
            :command (roslisp-msg-protocol:symbol-code
                      'robohow_common_msgs-srv:graspstabilitycontrol
                      :ctrl_stop)
            :context_id ,context-id)
           (when (eql (value state-fluent) :failed)
             (error 'grasp-stability-violated))
           return-value)))))
