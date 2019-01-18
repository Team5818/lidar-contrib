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
import com.armabot.lidar.impl.vl53l0x.Vl53l0xI2c;

import java.util.concurrent.TimeUnit;

/**
 * API for interacting with a VL53L0X range-finder.
 *
 * <p>
 * The suggested implementation to use is {@link Vl53l0xI2c}.
 * </p>
 */
public interface Vl53l0x extends AutoCloseable {

    enum VcselPeriodType {
        VCSEL_PERIOD_PRE_RANGE,
        VCSEL_PERIOD_FINAL_RANGE
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
     * This will change the address of the VL53L0X to match.
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
     * Initialize, check if we're talking to the right module, etc.
     *
     * @return {@code true} if the module is initialized
     */
    boolean initialize();

    @Override
    void close();

    boolean setSignalRateLimit(float limitMpcs);

    float getSignalRateLimit();

    boolean setMeasurementTimingBudget(long budgetMicrosec);

    long getMeasurementTimingBudget();

    boolean setVcselPulsePeriod(VcselPeriodType type, short periodPclks);

    short getVcselPulsePeriod(VcselPeriodType type);

    void startContinuous(long periodMilli);

    void stopContinuous();

    int readRangeContinuousMillimeters();

    int readRangeSingleMillimeters();

    void setTimeout(long timeout, TimeUnit unit);

    long getTimeout(TimeUnit unit);

    boolean timeoutOccurred();
}
