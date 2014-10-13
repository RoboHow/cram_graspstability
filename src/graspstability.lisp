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

(roslisp-utilities:register-ros-init-function config)

(define-policy grasp-stability-awareness (context-id worst-stability estimate-type object-name)
  "Policy for monitoring the grasp stability of the current grasp and
reacting accordingly."
  (:init (let* ((state-fluent (make-fluent :name "grasp-stability-state-fluent"
                                           :value :stopped))
                (state-subscriber
                  (roslisp:subscribe
                   *state-topic*
                   "robohow_common_msgs/GraspStability"
                   (lambda (msg)
                     (with-fields (context_id stability) msg
                       (declare (ignore context_id))
                       (format t "Got value: ~a~%" stability)
                       (cond
                         ((and
                           ;(string= context_id context-id)
                           (< stability worst-stability))
                          (format t "Grasp was bad~%")
                          (cpl:setf (value (cpl::policy-get 'state-fluent)) :failed))
                         (t (format
                             t "~a >= ~a~%" stability worst-stability))))))))
           (cpl::policy-setf 'state-fluent state-fluent)
           (cpl::policy-setf 'state-subscriber state-subscriber)
           (block connector
             (cpl:with-failure-handling
                 (((or sb-bsd-sockets:connection-refused-error roslisp::ros-rpc-error) (f)
                    (declare (ignore f))
                    (return-from connector)))
               (when (roslisp:wait-for-service *control-service* 2)
                 (roslisp:call-service
                  *control-service*
                  "robohow_common_msgs/GraspStabilityControl"
                  :command (roslisp-msg-protocol:symbol-code
                            'robohow_common_msgs-srv:graspstabilitycontrol-request
                            :start)
                  :context_id context-id
                  :object_id object-name
                  :estimate_type estimate-type)
                 (cpl:setf (value state-fluent) :running))))
           (cpl:sleep* 2)
           t))
  (:check (let ((state-fluent (cpl::policy-get 'state-fluent)))
            (wait-for state-fluent)
            (case (value state-fluent)
              (roslisp:ros-info "grasp stability" "Policy triggered: Grasp unstable.")
              (:failed t))))
  (:clean-up (let ((state-subscriber (cpl::policy-get 'state-subscriber)))
               (roslisp:unsubscribe state-subscriber)
               (block connector
                 (cpl:with-failure-handling
                     (((or sb-bsd-sockets:connection-refused-error roslisp::ros-rpc-error) (f)
                        (declare (ignore f))
                        (return-from connector)))
                   (when (roslisp:wait-for-service *control-service* 2)
                     (roslisp:call-service
                      *control-service*
                      "robohow_common_msgs/GraspStabilityControl"
                      :command (roslisp-msg-protocol:symbol-code
                                'robohow_common_msgs-srv:graspstabilitycontrol-request
                                :stop)
                      :context_id context-id)))))))

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
                    (< stability ,worst-stability))
                   (cpl:setf (value state-fluent) :failed))
                  (t (format
                      t "Reported grasp satisfies our requirements (~a >= ~a).~%" stability ,worst-stability))))))
       (let* ((state-subscriber (roslisp:subscribe
                                 *state-topic*
                                 "robohow_common_msgs/GraspStability"
                                 #'stability-state-callback)))
         (unwind-protect
              (progn
                (roslisp:call-service
                 *control-service*
                 "robohow_common_msgs/GraspStabilityControl"
                 :command (roslisp-msg-protocol:symbol-code
                           'robohow_common_msgs-srv:graspstabilitycontrol-request
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
            "robohow_common_msgs/GraspStabilityControl"
            :command (roslisp-msg-protocol:symbol-code
                      'robohow_common_msgs-srv:graspstabilitycontrol-request
                      :stop)
            :context_id ,context-id)
           (when (eql (value state-fluent) :failed)
             (error 'grasp-stability-violated))
           return-value)))))
