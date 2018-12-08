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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Objects;

import org.rivierarobotics.i2c.util.Preconditions;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.hal.I2CJNI;

/**
 * Drop-in replacement for Arduino's Wire class. Only works for master mode.
 * 
 * Avoids the higher level {@link I2C} since Wire is low level too.
 */
public class Wire {

    private final Port port;

    public Wire(Port port) {
        this.port = Objects.requireNonNull(port, "port");
    }

    public Port getPort() {
        return port;
    }

    // Wire works by building the arrays using begin/end transmission wrappers.
    private static final int BUFFER_LENGTH = 32;
    private boolean transmitting = false;
    private ByteBuffer rxBuffer = ByteBuffer.allocateDirect(BUFFER_LENGTH).order(ByteOrder.BIG_ENDIAN);
    private ByteBuffer txBuffer = ByteBuffer.allocateDirect(BUFFER_LENGTH).order(ByteOrder.BIG_ENDIAN);
    private byte txAddress = -1;

    /**
     * Initialize this {@link Wire} object. Essentially just calls
     * {@link I2CJNI#i2CInitialize(int)}.
     */
    public void begin() {
        resetRxBuffer();

        resetTxBuffer();

        I2CJNI.i2CInitialize(port.value);
    }

    private void resetRxBuffer() {
        rxBuffer.clear();
    }

    private void resetTxBuffer() {
        txBuffer.clear();
    }

    /**
     * Start buffering for a transmission to {@code address}.
     * 
     * @param address
     *            the address to send to
     */
    public void beginTransmission(byte address) {
        Preconditions.checkArgument(address >= 0, "Address may not be null");
        transmitting = true;
        txAddress = address;
        resetTxBuffer();
    }

    private void checkTransmitting() {
        Preconditions.checkState(transmitting, "Not transmitting, please use beginTransmission()");
    }

    /**
     * End the current transmission and write it.
     * 
     * @return {@code true} if successfully transmitted
     */
    public boolean endTransmission() {
        checkTransmitting();
        boolean success = I2CJNI.i2CWrite(port.value, txAddress, txBuffer, (byte) txBuffer.position()) >= 0;

        resetTxBuffer();
        transmitting = false;

        return success;
    }

    private void checkWriteable(int amt) {
        Preconditions.checkState(txBuffer.remaining() >= amt, "No more room in the buffer");
    }

    public void write(byte data) {
        checkTransmitting();
        checkWriteable(Byte.BYTES);
        txBuffer.put(data);
    }

    public void writeShort(short data) {
        checkTransmitting();
        checkWriteable(Short.BYTES);
        txBuffer.putShort(data);
    }

    public void writeInt(int data) {
        checkTransmitting();
        checkWriteable(Integer.BYTES);
        txBuffer.putInt(data);
    }

    public void write(byte[] data) {
        checkTransmitting();
        Preconditions.checkState(txBuffer.remaining() >= data.length, "No more room in the buffer");
        txBuffer.put(data);
    }

    public void requestFrom(byte address, byte amount) {
        I2CJNI.i2CRead(port.value, address, rxBuffer, amount);
        rxBuffer.position(0).limit(amount);
    }

    private void checkReadable(int amt) {
        Preconditions.checkState(rxBuffer.remaining() >= amt, "No more data available");
    }

    public byte read() {
        checkReadable(Byte.BYTES);
        return rxBuffer.get();
    }

    public short readShort() {
        checkReadable(Short.BYTES);
        return rxBuffer.getShort();
    }

    public int readInt() {
        checkReadable(Integer.BYTES);
        return rxBuffer.getInt();
    }

}
