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
import edu.wpi.first.hal.I2CJNI;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Objects;

/**
 * Drop-in replacement for Arduino's Wire class. Only works for master mode.
 * <p>
 * Avoids the higher level I2C since Wire is low level too.
 */
public class Wire implements AutoCloseable {

    private final Port port;

    public Wire(Port port) {
        this.port = Objects.requireNonNull(port, "port");
    }

    public Port getPort() {
        return port;
    }

    // Wire works by building the arrays using begin/end transmission wrappers.
    private static final int BUFFER_LENGTH = 32;
    private boolean open = false;
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

        I2CJNI.i2CInitialize(port.value());
        open = true;
    }

    @Override
    public void close() {
        I2CJNI.i2CClose(port.value());
        open = false;
    }

    private void resetRxBuffer() {
        rxBuffer.clear();
    }

    private void resetTxBuffer() {
        txBuffer.clear();
    }

    private void checkOpen() {
        Preconditions.checkState(open, "Not open, please call begin() first");
    }

    /**
     * Start buffering for a transmission to {@code address}.
     *
     * @param address the address to send to
     */
    public void beginTransmission(byte address) {
        Preconditions.checkArgument(address >= 0, "Address may not be null");
        checkOpen();
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
        checkOpen();
        checkTransmitting();
        boolean success = I2CJNI.i2CWrite(port.value(), txAddress, txBuffer, (byte) txBuffer.position()) >= 0;

        resetTxBuffer();
        transmitting = false;

        return success;
    }

    private void checkWriteable(int amt) {
        Preconditions.checkState(txBuffer.remaining() >= amt, "No more room in the buffer");
    }

    public void write(short data) {
        checkOpen();
        checkTransmitting();
        checkWriteable(Byte.BYTES);
        txBuffer.put((byte) (data & 0xFF));
    }

    public void writeShort(int data) {
        checkOpen();
        checkTransmitting();
        checkWriteable(Short.BYTES);
        txBuffer.put((byte) (data >>> 8 & 0xFF));
        txBuffer.put((byte) (data & 0xFF));
    }

    public void writeInt(long data) {
        checkOpen();
        checkTransmitting();
        checkWriteable(Integer.BYTES);
        txBuffer.put((byte) (data >>> 24 & 0xFF));
        txBuffer.put((byte) (data >>> 16 & 0xFF));
        txBuffer.put((byte) (data >>> 8 & 0xFF));
        txBuffer.put((byte) (data & 0xFF));
    }

    public void write(byte[] data) {
        checkOpen();
        checkTransmitting();
        Preconditions.checkState(txBuffer.remaining() >= data.length, "No more room in the buffer");
        txBuffer.put(data);
    }

    public void requestFrom(byte address, byte amount, boolean continueTransmission) {
        Preconditions.checkArgument(0 <= amount && amount <= BUFFER_LENGTH,
                "amount must be within buffer bounds");
        checkOpen();
        if (continueTransmission) {
            checkTransmitting();
            I2CJNI.i2CTransaction(port.value(), address,
                    txBuffer, (byte) txBuffer.position(),
                    rxBuffer, amount);
            resetTxBuffer();
            transmitting = false;
        } else {
            I2CJNI.i2CRead(port.value(), address,
                    rxBuffer, amount);
        }
        rxBuffer.position(0).limit(amount);
    }

    private void checkReadable(int amt) {
        Preconditions.checkState(rxBuffer.remaining() >= amt, "No more data available");
    }

    public short read() {
        checkOpen();
        checkReadable(Byte.BYTES);
        return (short) Byte.toUnsignedInt(rxBuffer.get());
    }

    public int readShort() {
        checkOpen();
        checkReadable(Short.BYTES);
        return Short.toUnsignedInt(rxBuffer.getShort());
    }

    public long readInt() {
        checkOpen();
        checkReadable(Integer.BYTES);
        return Integer.toUnsignedLong(rxBuffer.getInt());
    }

    public void read(byte[] out) {
        checkOpen();
        checkReadable(out.length);
        rxBuffer.get(out);
    }

}
