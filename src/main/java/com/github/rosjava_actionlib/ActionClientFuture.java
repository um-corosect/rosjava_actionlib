package com.github.rosjava_actionlib;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.message.Message;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class ActionClientFuture<T_GOAL extends Message, T_FEEDBACK extends Message, T_RESULT extends Message>
        implements ActionFuture<T_GOAL, T_FEEDBACK, T_RESULT>, ActionClientListener<T_FEEDBACK, T_RESULT> {

    final GoalID goalid;
    final ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> ac;
    ClientGoalManager goalManager = new ClientGoalManager(new ActionGoal<T_GOAL>());
    T_FEEDBACK latestFeedback = null;
    T_RESULT result = null;

    private static Log log = LogFactory.getLog(ActionClientFuture.class);

    static <T_GOAL extends Message, T_FEEDBACK extends Message, T_RESULT extends Message>
    ActionFuture<T_GOAL, T_FEEDBACK, T_RESULT>
    createFromGoal(ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> ac, T_GOAL goal) {


        GoalID goalId = ac.getGoalId(goal);
        ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> ret = new ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT>(ac, goalId);
        if (ac.isActive()) {
            log.warn("current goal STATE:" + ac.getGoalState() + "=" + ac.getGoalState().getValue());
        }
        ret.goalManager.setGoal(goal);
        ac.sendGoalWire(goal);
        ac.attachListener(ret);
        return ret;

    }

    private ActionClientFuture(ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> ac, GoalID id) {
        this.ac = ac;
        this.goalid = id;
    }

    @Override
    public T_FEEDBACK getLatestFeedback() {
        return latestFeedback;
    }

    @Override
    public ClientState getCurrentState() {
        return goalManager.stateMachine.getState();
    }

    @Override
    public boolean cancel(boolean bln) {
        ac.sendCancel(goalid);
        goalManager.cancelGoal();
        return true;
    }

    @Override
    public boolean isCancelled() {
        if (goalManager.stateMachine.isRunning()) {
            return result == null;
        } else {
            return false;
        }
    }

    @Override
    public boolean isDone() {
        return !goalManager.stateMachine.isRunning();
    }

    @Override
    public T_RESULT get() throws InterruptedException, ExecutionException {
        while (goalManager.stateMachine.isRunning()) {
            Thread.sleep(100);
        }
        disconnect();
        return result;
    }

    @Override
    public T_RESULT get(long l, TimeUnit tu) throws InterruptedException, ExecutionException, TimeoutException {
        long timeout = System.currentTimeMillis() + tu.toMillis(l);
        while (goalManager.stateMachine.isRunning()) {
            if (timeout > System.currentTimeMillis()) {
                throw new TimeoutException();
            }
            Thread.sleep(50);
        }
        disconnect();
        return result;
    }

    @Override
    public void resultReceived(T_RESULT msg) {
        ActionResult r = new ActionResult(msg);
        log.debug("got result message " + r.getGoalStatusMessage().getGoalId().getId());
        if (!r.getGoalStatusMessage().getGoalId().getId().equals(goalid.getId())) {
            log.fatal("wrong id, waiting for " + goalid.getId());
            return;
        }

        goalManager.updateStatus(r.getGoalStatusMessage().getStatus());
        goalManager.resultReceived();

        result = msg;
        disconnect();
    }

    @Override
    public void feedbackReceived(T_FEEDBACK msg) {
        ActionFeedback f = new ActionFeedback(msg);
        if (!f.getGoalStatusMessage().getGoalId().getId().equals(goalid.getId())) {
            return;
        }

        goalManager.updateStatus(f.getGoalStatusMessage().getStatus());
        latestFeedback = msg;
    }

    @Override
    public void statusReceived(GoalStatusArray status) {
        for (GoalStatus a : status.getStatusList()) {
            if (!goalid.getId().equals(a.getGoalId().getId())) {
                continue;
            }

            goalManager.updateStatus(a.getStatus());

        }
    }

    private void disconnect() {
        ac.detachListener(this);
    }

    @Override
    public Future<Boolean> toBooleanFuture() {

        final ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> self = this;

        return new Future<Boolean>() {
            @Override
            public boolean cancel(boolean bln) {
                return self.cancel(bln);
            }

            @Override
            public boolean isCancelled() {
                return self.isCancelled();
            }

            @Override
            public boolean isDone() {
                return self.isDone();
            }

            @Override
            public Boolean get() throws InterruptedException, ExecutionException {
                return self.get() != null;
            }

            @Override
            public Boolean get(long l, TimeUnit tu) throws InterruptedException, ExecutionException, TimeoutException {
                return this.get(l, tu) != null;
            }
        };

    }

    @Override
    public Future<Void> toVoidFuture() {

        final ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> self = this;

        return new Future<Void>() {
            @Override
            public boolean cancel(boolean bln) {
                return self.cancel(bln);
            }

            @Override
            public boolean isCancelled() {
                return self.isCancelled();
            }

            @Override
            public boolean isDone() {
                return self.isDone();
            }

            @Override
            public Void get() throws InterruptedException, ExecutionException {
                self.get();
                return null;
            }

            @Override
            public Void get(long l, TimeUnit tu) throws InterruptedException, ExecutionException, TimeoutException {
                self.get(l, tu);
                return null;
            }
        };

    }

}
