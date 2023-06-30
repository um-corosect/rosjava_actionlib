package com.github.rosjava_actionlib;

import java.util.HashMap;
import java.util.Map;

public enum ClientState {
    ERROR(-3),
    INVALID_TRANSITION(-2),
    NO_TRANSITION(-1),
    WAITING_FOR_GOAL_ACK(0),
    PENDING(1),
    ACTIVE(2),
    WAITING_FOR_RESULT(3),
    WAITING_FOR_CANCEL_ACK(4),
    RECALLING(5),
    PREEMPTING(6),
    DONE(7),
    LOST(8),
    UNKNOWN_STATE(99);

    private final int Value;

    public Integer getValue() {
        return Value;
    }

    ClientState(int value) {
        Value = value;
    }

    // Mapping states to state id
    private static final Map<Integer, ClientState> _map = new HashMap<Integer, ClientState>();

    static {
        for (ClientState state : ClientState.values())
            _map.put(state.Value, state);
    }

    /**
     * Get state from value
     *
     * @param value Value
     * @return state
     */
    public static ClientState from(int value) {
        return _map.getOrDefault(value, UNKNOWN_STATE);
    }
}
