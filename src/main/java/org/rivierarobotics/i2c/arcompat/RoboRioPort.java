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

package org.rivierarobotics.i2c.arcompat;

/**
 * Ports available on the RoboRIO board.
 */
public enum RoboRioPort implements Port {
    /**
     * The I2C port on the RoboRIO's board.
     */
    ONBOARD(0),
    /**
     * The I2C port on the MXP output.
     */
    MXP(1);

    private final int port;

    RoboRioPort(int port) {
        this.port = port;
    }

    @Override
    public int value() {
        return port;
    }
}
