package com.github.rosjava_actionlib;

import org.ros.internal.message.Message;

import java.util.concurrent.Future;

public interface ActionFuture<T_ACTION_GOAL extends Message, T_ACTION_FEEDBACK extends Message, T_ACTION_RESULT extends Message> extends Future<T_ACTION_RESULT> {

    T_ACTION_FEEDBACK getLatestFeedback();
    ClientState getCurrentState();

    Future<Void> toVoidFuture();

    Future<Boolean> toBooleanFuture();
}

