package com.armabot.lidar.impl;

import com.armabot.lidar.api.Error;

public abstract class ErrorBase<T> implements Error<T> {

    private final String code;
    private final T data;

    protected ErrorBase(String code, T data) {
        this.code = code;
        this.data = data;
    }

    @Override
    public final String code() {
        return code;
    }

    @Override
    public T data() {
        return data;
    }

    protected String formatMessage() {
        return data.toString();
    }

    @Override
    public String toString() {
        return "[" + code + "] " + formatMessage();
    }
}
