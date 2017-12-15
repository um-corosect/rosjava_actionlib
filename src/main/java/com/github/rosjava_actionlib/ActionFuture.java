package com.github.rosjava_actionlib;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import org.ros.internal.message.Message;

public interface ActionFuture<T_ACTION_GOAL extends Message, T_ACTION_FEEDBACK extends Message, T_ACTION_RESULT extends Message> extends Future<T_ACTION_RESULT> {

    T_ACTION_FEEDBACK getLatestFeedback();
}

