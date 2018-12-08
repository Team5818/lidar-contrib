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

import java.util.concurrent.TimeUnit;

public interface Vl53lx {

    byte DEFAULT_ADDRESS = 0x29;

    byte getAddress();

    void setAddress(byte address);

    PololuI2c getI2c();

    /**
     * Initialize, check if we're talking to the right module, etc.
     *
     * @return {@code true} if the module is initialized
     */
    boolean initialize();

    DistanceMode getDistanceMode();

    boolean setDistanceMode(DistanceMode mode);

    int getMeasurementTimingBudget();

    boolean setMeasurementTimingBudget(int budgetMicro);

    void startContinuous(int periodMillis);

    void stopContinuous();

    short read();

    boolean dataReady();

    void setTimeout(long timeout, TimeUnit unit);

    long getTimeout(TimeUnit unit);

    boolean timeoutOccurred();

}
