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
 * Simplistic API for registers on the I2C target. Usually implemented by an
 * enum for easy iteration.
 */
public interface Register {
    
    interface Bound {
        boolean write(byte value);
        boolean write16Bit(short value);
        boolean write32Bit(int value);
        byte read();
        short read16Bit();
        int read32Bit();
    }

    short address();
    
    default Bound on(PololuI2c i2c) {
        return new RegisterBinding(address(), i2c);
    }

}
