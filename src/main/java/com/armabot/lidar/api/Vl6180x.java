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


import com.armabot.lidar.arcompat.PololuI2c;
import com.armabot.lidar.impl.vl6180x.Vl6180xI2c;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

/**
 * API for interacting with a VL6180X range-finder.
 *
 * <p>
 * The suggested implementation to use is {@link Vl6180xI2c}.
 * </p>
 */
public interface Vl6180x extends AutoCloseable {

    /**
     * Scaling values, for 1x, 2x, and 3x.
     */
    enum Scaling {
        ONE_TIMES,
        TWO_TIMES,
        THREE_TIMES;

        public final int amount() {
            return ordinal() + 1;
        }
    }

    byte DEFAULT_ADDRESS = 0x29;

    /**
     * @return the current address
     */
    byte getAddress();

    /**
     * Sets the current address to communicate with.
     *
     * <p>
     * This will change the address of the VL6180X to match.
     * If you only want to change the address of this object,
     * use {@link PololuI2c#setAddress(byte)}.
     * </p>
     *
     * @param address the new address
     */
    void setAddress(byte address);

    /**
     * @return the underlying I2C communication helper
     */
    PololuI2c getI2c();

    /**
     * Initialize. Check if we're talking to the right module, etc.
     *
     * @return {@link Optional#empty()} if the module is initialized
     */
    Optional<Error<?>> initialize();

    /**
     * Configure sensible defaults.
     */
    void configureDefault();

    @Override
    void close();

    /**
     * Sets range scaling factor. The sensor uses 1x scaling by default, giving range measurements
     * in units of mm. Increasing the scaling to 2x or 3x makes it give raw values in units of 2 mm
     * or 3 mm instead. In other words, a bigger scaling factor increases the sensor's potential
     * maximum range but reduces its resolution.
     */
    void setScaling(Scaling scaling);

    Scaling getScaling();

    short readRangeSingle();

    default int readRangeSingleMillimeters() {
        return getScaling().amount() * readRangeSingle();
    }

    int readAmbientSingle();

    void startRangeContinuous(int period);

    void startAmbientContinuous(int period);

    void startInterleavedContinuous(int period);

    void stopContinuous();

    boolean dataReadyRange();

    /**
     * Read range data, if there is {@linkplain #dataReadyRange() data ready}.
     *
     * @throws IllegalStateException if there is no data ready
     */
    short readRangeContinuous();

    /**
     * Read range data, if there is {@linkplain #dataReadyRange() data ready}.
     *
     * @throws IllegalStateException if there is no data ready
     */
    default int readRangeContinuousMillimeters() {
        return getScaling().amount() * readRangeContinuous();
    }

    boolean dataReadyAmbient();

    /**
     * Read ambient data, if there is {@linkplain #dataReadyAmbient() data ready}.
     *
     * @throws IllegalStateException if there is no data ready
     */
    int readAmbientContinuous();

    /**
     * Sets the timeout for a response from the VL6180X unit.
     */
    void setTimeout(long timeout, TimeUnit unit);

    /**
     * Gets the current timeout, in the requested unit.
     */
    long getTimeout(TimeUnit unit);

    boolean timeoutOccurred();
}
