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

package com.armabot.lidar.arcompat;

import com.armabot.lidar.util.Preconditions;

/**
 * Pololu's APIs for Arduino use extended I2C protocol frequently. This class
 * implements those using {@link Wire}.
 */
public class PololuI2c implements AutoCloseable {

    public enum Addressing {
        EIGHT_BIT {
            @Override
            void writeAddress(int address, Wire wire) {
                wire.write((short) address);
            }
        },
        SIXTEEN_BIT {
            @Override
            void writeAddress(int address, Wire wire) {
                wire.writeShort(address);
            }
        },
        ;
         abstract void writeAddress(int address, Wire wire);
    }

    public static PololuI2c create(Port port, Addressing addressing) {
        return new PololuI2c(new Wire(port), addressing);
    }

    private enum Status {
        UNSET(null), SUCCESS(true), FAILURE(false);

        public final Boolean value;

        Status(Boolean value) {
            this.value = value;
        }
    }

    private final Wire wire;
    private final Addressing addressing;
    private byte address = -1;
    private Status lastStatus = Status.UNSET;

    private PololuI2c(Wire wire, Addressing addressing) {
        this.wire = wire;
        this.addressing = addressing;
        wire.begin();
    }

    @Override
    public void close() {
        wire.close();
    }

    public Wire getWire() {
        return wire;
    }

    public byte getAddress() {
        Preconditions.checkState(address >= 0, "Address not set.");
        return address;
    }

    public void setAddress(byte address) {
        Preconditions.checkArgument(address >= 0, "Address must be positive");
        this.address = address;
    }

    public boolean wasLastOpSuccessful() {
        Preconditions.checkState(lastStatus != Status.UNSET, "No operation performed yet");
        return lastStatus.value;
    }

    public void beginTransmission() {
        byte address = getAddress();
        wire.beginTransmission(address);
    }

    public boolean endTransmission() {
        boolean success = wire.endTransmission();
        lastStatus = success ? Status.SUCCESS : Status.FAILURE;
        return success;
    }

    public boolean writeReg(short reg, short value) {
        beginTransmission();
        addressing.writeAddress(reg, wire);
        wire.write(value);
        return endTransmission();
    }

    public boolean writeReg16Bit(short reg, int value) {
        beginTransmission();
        addressing.writeAddress(reg, wire);
        wire.writeShort(value);
        return endTransmission();
    }

    public boolean writeReg32Bit(short reg, long value) {
        beginTransmission();
        addressing.writeAddress(reg, wire);
        wire.writeInt(value);
        return endTransmission();
    }

    public boolean writeRegMulti(short reg, byte[] value) {
        beginTransmission();
        addressing.writeAddress(reg, wire);
        wire.write(value);
        return endTransmission();
    }

    public void askForRegValue(short reg) {
        beginTransmission();
        addressing.writeAddress(reg, wire);
        endTransmission();
    }

    public void request(int amount) {
        wire.requestFrom(getAddress(), (byte) amount, false);
    }

    public short readReg(short reg) {
        askForRegValue(reg);
        request(Byte.BYTES);
        return wire.read();
    }

    public int readReg16Bit(short reg) {
        askForRegValue(reg);
        request(Short.BYTES);
        return wire.readShort();
    }

    public long readReg32Bit(short reg) {
        askForRegValue(reg);
        request(Integer.BYTES);
        return wire.readInt();
    }

    public void readRegMulti(short reg, byte[] out) {
        askForRegValue(reg);
        request(out.length);
        wire.read(out);
    }
}
