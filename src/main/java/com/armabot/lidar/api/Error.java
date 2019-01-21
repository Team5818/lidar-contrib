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

package com.armabot.lidar.api;

import java.util.Optional;
import java.util.function.Consumer;

/**
 * Represents some kind of error.
 *
 * <p>
 * Each error has a code and some data. The string representation
 * is also suitable for easy reporting.
 * </p>
 *
 * <p>
 * Many API methods return {@code Optional<Error<?>>}. This should be
 * handled by {@link Optional#ifPresent(Consumer)} in most cases,
 * where you throw an exception better detailing the error for your
 * use case.
 * </p>
 */
public interface Error<T> {

    String code();

    T data();

    /**
     * Provides a reportable representation of the error.
     *
     * <p>Typically this will look like {@code [$code] $message}.</p>
     */
    @Override
    String toString();

}
