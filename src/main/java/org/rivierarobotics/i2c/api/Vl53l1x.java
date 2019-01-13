/*
 * This file is part of pololu-frc-contrib, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
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

package org.rivierarobotics.i2c.api;

import org.rivierarobotics.i2c.arcompat.PololuI2c;
import org.rivierarobotics.i2c.impl.vl53l1x.DistanceMode;
import org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xI2c;

import java.util.concurrent.TimeUnit;

/**
 * API for interacting with a VL53L1X range-finder.
 *
 * <p>
 * The suggested implementation to use is {@link Vl53l1xI2c}.
 * </p>
 */
public interface Vl53l1x {

    /**
     * The default address that VL53L1X has.
     */
    byte DEFAULT_ADDRESS = 0x29;

    /**
     * @return the current address
     */
    byte getAddress();

    /**
     * Sets the current address to communicate with.
     *
     * <p>
     * This will change the address of the VL53L1X to match.
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

    /**
     * @return the current distance mode
     */
    DistanceMode getDistanceMode();

    /**
     * Sets the distance mode.
     *
     * @return {@code true} if it was successfully set
     */
    boolean setDistanceMode(DistanceMode mode);

    /**
     * @return the current measurement timing budget, in microseconds
     */
    int getMeasurementTimingBudget();

    /**
     * Sets the measurement timing budget, in microseconds.
     *
     * @return {@code true} if it was successfully set
     */
    boolean setMeasurementTimingBudget(int budgetMicro);

    /**
     * Starts continuous reading. Use {@link #read()} to retrieve values.
     */
    void startContinuous(int periodMillis);

    /**
     * Stops continuous reading.
     */
    void stopContinuous();

    /**
     * Reads the next distance measurement.
     *
     * <p>
     * You should verify that this data is valid by checking
     * {@linkplain #timeoutOccurred() if a timeout has occurred}
     * </p>
     *
     * @return the next measurement, in millimeters
     */
    int read();

    /**
     * @return if there is data available
     */
    boolean dataReady();

    /**
     * Sets the timeout for a response from the VL53L1X unit.
     */
    void setTimeout(long timeout, TimeUnit unit);

    /**
     * Gets the current timeout, in the requested unit.
     */
    long getTimeout(TimeUnit unit);

    boolean timeoutOccurred();

}
