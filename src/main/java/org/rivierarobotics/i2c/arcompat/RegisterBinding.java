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

class RegisterBinding implements Register.Bound {

    private final short reg;
    private final PololuI2c i2c;

    RegisterBinding(short reg, PololuI2c i2c) {
        this.reg = reg;
        this.i2c = i2c;
    }

    @Override
    public boolean write(short value) {
        return i2c.writeReg(reg, value);
    }

    @Override
    public boolean write16Bit(int value) {
        return i2c.writeReg16Bit(reg, value);
    }

    @Override
    public boolean write32Bit(long value) {
        return i2c.writeReg32Bit(reg, value);
    }

    @Override
    public short read() {
        return i2c.readReg(reg);
    }

    @Override
    public int read16Bit() {
        return i2c.readReg16Bit(reg);
    }

    @Override
    public long read32Bit() {
        return i2c.readReg32Bit(reg);
    }

}
