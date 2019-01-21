/*
 * This file is part of lidar-contrib, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Armabot <https://www.armabot.com>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
