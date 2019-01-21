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

package com.armabot.lidar.impl.vl6180x;

import com.armabot.lidar.api.Error;
import com.armabot.lidar.api.Vl6180x;
import com.armabot.lidar.arcompat.PololuI2c;
import com.armabot.lidar.arcompat.Port;
import com.armabot.lidar.impl.errors.IncorrectModelId;

import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

import static com.armabot.lidar.impl.vl6180x.Calculations.constrain;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.I2C_SLAVE__DEVICE_ADDRESS;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.IDENTIFICATION__MODEL_ID;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.INTERLEAVED_MODE__ENABLE;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.RANGE_SCALER;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.READOUT__AVERAGING_SAMPLE_PERIOD;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.RESULT__ALS_VAL;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.RESULT__INTERRUPT_STATUS_GPIO;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.RESULT__RANGE_VAL;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSALS__ANALOGUE_GAIN;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSALS__INTEGRATION_PERIOD;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSALS__INTERMEASUREMENT_PERIOD;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSALS__START;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__CROSSTALK_VALID_HEIGHT;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__INTERMEASUREMENT_PERIOD;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__MAX_CONVERGENCE_TIME;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__PART_TO_PART_RANGE_OFFSET;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__RANGE_CHECK_ENABLES;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__START;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__VHV_RECALIBRATE;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSRANGE__VHV_REPEAT_RATE;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSTEM__FRESH_OUT_OF_RESET;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSTEM__INTERRUPT_CLEAR;
import static com.armabot.lidar.impl.vl6180x.Vl6180xReg.SYSTEM__INTERRUPT_CONFIG_GPIO;
import static com.armabot.lidar.util.Preconditions.checkArgument;
import static com.armabot.lidar.util.Preconditions.checkState;

/**
 * A near-direct port of the
 * <a href="https://github.com/pololu/vl6180x-arduino">vl6180x-arduino</a>
 * library.
 */
// The original library is licensed under the terms in LICENSE-vl6180x.txt
public class Vl6180xI2c implements Vl6180x {
    private static final int MODEL_ID = 0xB4;

    private final PololuI2c i2c;
    private long timeout;
    private long timeoutStart;
    private boolean didTimeout;
    private Scaling scaling = Scaling.ONE_TIMES;
    private short ptpOffset;

    public Vl6180xI2c(Port port) {
        this.i2c = PololuI2c.create(port, PololuI2c.Addressing.SIXTEEN_BIT);
        i2c.setAddress(DEFAULT_ADDRESS);
    }

    @Override
    public void close() {
        i2c.close();
    }

    @Override
    public byte getAddress() {
        return i2c.getAddress();
    }

    @Override
    public void setAddress(byte address) {
        if (address == getAddress()) {
            return;
        }
        I2C_SLAVE__DEVICE_ADDRESS.on(i2c).write((byte) (address & 0x7F));
        i2c.setAddress(address);
    }

    @Override
    public Optional<Error<?>> initialize() {
        int modelId = IDENTIFICATION__MODEL_ID.on(i2c).read();
        if (modelId != MODEL_ID) {
            return Optional.of(IncorrectModelId.of("VL6180X", MODEL_ID, modelId));
        }

        ptpOffset = SYSRANGE__PART_TO_PART_RANGE_OFFSET.on(i2c).read();

        if (SYSTEM__FRESH_OUT_OF_RESET.on(i2c).read() == 1) {
            scaling = Scaling.ONE_TIMES;

            i2c.writeReg((short) 0x207, (short) 0x01);
            i2c.writeReg((short) 0x208, (short) 0x01);
            i2c.writeReg((short) 0x096, (short) 0x00);
            i2c.writeReg((short) 0x097, (short) 0xFD); // RANGE_SCALER = 253
            i2c.writeReg((short) 0x0E3, (short) 0x00);
            i2c.writeReg((short) 0x0E4, (short) 0x04);
            i2c.writeReg((short) 0x0E5, (short) 0x02);
            i2c.writeReg((short) 0x0E6, (short) 0x01);
            i2c.writeReg((short) 0x0E7, (short) 0x03);
            i2c.writeReg((short) 0x0F5, (short) 0x02);
            i2c.writeReg((short) 0x0D9, (short) 0x05);
            i2c.writeReg((short) 0x0DB, (short) 0xCE);
            i2c.writeReg((short) 0x0DC, (short) 0x03);
            i2c.writeReg((short) 0x0DD, (short) 0xF8);
            i2c.writeReg((short) 0x09F, (short) 0x00);
            i2c.writeReg((short) 0x0A3, (short) 0x3C);
            i2c.writeReg((short) 0x0B7, (short) 0x00);
            i2c.writeReg((short) 0x0BB, (short) 0x3C);
            i2c.writeReg((short) 0x0B2, (short) 0x09);
            i2c.writeReg((short) 0x0CA, (short) 0x09);
            i2c.writeReg((short) 0x198, (short) 0x01);
            i2c.writeReg((short) 0x1B0, (short) 0x17);
            i2c.writeReg((short) 0x1AD, (short) 0x00);
            i2c.writeReg((short) 0x0FF, (short) 0x05);
            i2c.writeReg((short) 0x100, (short) 0x05);
            i2c.writeReg((short) 0x199, (short) 0x05);
            i2c.writeReg((short) 0x1A6, (short) 0x1B);
            i2c.writeReg((short) 0x1AC, (short) 0x3E);
            i2c.writeReg((short) 0x1A7, (short) 0x1F);
            i2c.writeReg((short) 0x030, (short) 0x00);

            SYSTEM__FRESH_OUT_OF_RESET.on(i2c).write((short) 0);
        } else {
            // Sensor has already been initialized, so try to get scaling settings by
            // reading registers.

            int s = RANGE_SCALER.on(i2c).read16Bit();

            this.scaling = Scaling.ONE_TIMES;
            for (Scaling scaling : Scaling.values()) {
                if (s == SCALING_VALUES.get(scaling)) {
                    this.scaling = scaling;
                }
            }

            // Adjust the part-to-part range offset value read earlier to account for
            // existing scaling. If the sensor was already in 2x or 3x scaling mode,
            // precision will be lost calculating the original (1x) offset, but this can
            // be resolved by resetting the sensor and Arduino again.
            ptpOffset *= scaling.amount();
        }

        return Optional.empty();
    }

    @Override
    public PololuI2c getI2c() {
        return i2c;
    }

    private static final Map<Scaling, Short> SCALING_VALUES;

    static {
        SCALING_VALUES = new EnumMap<>(Scaling.class);
        SCALING_VALUES.put(Scaling.ONE_TIMES, (short) 253);
        SCALING_VALUES.put(Scaling.TWO_TIMES, (short) 127);
        SCALING_VALUES.put(Scaling.THREE_TIMES, (short) 84);
    }

    private static final short DEFAULT_CROSSTALK_VALID_HEIGHT = 20;

    @Override
    public void setScaling(Scaling scaling) {
        this.scaling = scaling;
        RANGE_SCALER.on(i2c).write16Bit(SCALING_VALUES.get(scaling));

        int scalingAmt = scaling.amount();
        // apply scaling on part-to-part offset
        SYSRANGE__PART_TO_PART_RANGE_OFFSET.on(i2c).write((short) (ptpOffset / scalingAmt));

        // apply scaling on CrossTalkValidHeight
        SYSRANGE__CROSSTALK_VALID_HEIGHT.on(i2c).write((short) (DEFAULT_CROSSTALK_VALID_HEIGHT / scalingAmt));

        // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

        // enable early convergence estimate only at 1x scaling
        short rce = SYSRANGE__RANGE_CHECK_ENABLES.on(i2c).read();
        SYSRANGE__RANGE_CHECK_ENABLES.on(i2c).write((short) ((rce & 0xFE) |
                (scaling == Scaling.ONE_TIMES ? 1 : 0)));
    }

    @Override
    public Scaling getScaling() {
        return scaling;
    }

    @Override
    public void configureDefault() {
        // "Recommended : Public registers"

        // readout__averaging_sample_period = 48
        READOUT__AVERAGING_SAMPLE_PERIOD.on(i2c).write((short) 0x30);

        // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to Table 14 in datasheet)
        SYSALS__ANALOGUE_GAIN.on(i2c).write((short) 0x46);

        // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
        SYSRANGE__VHV_REPEAT_RATE.on(i2c).write((short) 0xFF);

        // sysals__integration_period = 99 (100 ms)
        // AN4545 incorrectly recommends writing to register 0x040; 0x63 should go in the lower byte, which is register 0x041.
        SYSALS__INTEGRATION_PERIOD.on(i2c).write16Bit(0x0063);

        // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
        SYSRANGE__VHV_RECALIBRATE.on(i2c).write((short) 0x01);


        // "Optional: Public registers"

        // sysrange__intermeasurement_period = 9 (100 ms)
        SYSRANGE__INTERMEASUREMENT_PERIOD.on(i2c).write((short) 0x09);

        // sysals__intermeasurement_period = 49 (500 ms)
        SYSALS__INTERMEASUREMENT_PERIOD.on(i2c).write((short) 0x31);

        // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
        SYSTEM__INTERRUPT_CONFIG_GPIO.on(i2c).write((short) 0x24);


        // Reset other settings to power-on defaults

        // sysrange__max_convergence_time = 49 (49 ms)
        SYSRANGE__MAX_CONVERGENCE_TIME.on(i2c).write((short) 0x31);

        // disable interleaved mode
        INTERLEAVED_MODE__ENABLE.on(i2c).write((short) 0);

        // reset range scaling factor to 1x
        setScaling(Scaling.ONE_TIMES);
    }

    @Override
    public short readRangeSingle() {
        SYSRANGE__START.on(i2c).write((short) 0x01);
        startTimeout();
        while (!dataReadyRange()) {
            if (currentlyTimedOut()) {
                setTimeoutFlag();
                return 0;
            }
        }
        return readRangeContinuous();
    }

    @Override
    public int readAmbientSingle() {
        SYSALS__START.on(i2c).write((short) 0x01);
        while (!dataReadyAmbient()) {
            if (currentlyTimedOut()) {
                setTimeoutFlag();
                return 0;
            }
        }
        return readAmbientContinuous();
    }

    @Override
    public void startRangeContinuous(int period) {
        short period_reg = (short) (period / 10 - 1);
        period_reg = constrain(period_reg, (short) 0, (short) 254);

        SYSRANGE__INTERMEASUREMENT_PERIOD.on(i2c).write(period_reg);
        SYSRANGE__START.on(i2c).write((short) 0x03);
    }

    @Override
    public void startAmbientContinuous(int period) {
        short period_reg = (short) (period / 10 - 1);
        period_reg = constrain(period_reg, (short) 0, (short) 254);

        SYSALS__INTERMEASUREMENT_PERIOD.on(i2c).write(period_reg);
        SYSALS__START.on(i2c).write((short) 0x03);
    }

    @Override
    public void startInterleavedContinuous(int period) {
        short period_reg = (short) (period / 10 - 1);
        period_reg = constrain(period_reg, (short) 0, (short) 254);

        INTERLEAVED_MODE__ENABLE.on(i2c).write((short) 1);
        SYSALS__INTERMEASUREMENT_PERIOD.on(i2c).write(period_reg);
        SYSALS__START.on(i2c).write((short) 0x03);
    }

    @Override
    public void stopContinuous() {
        SYSRANGE__START.on(i2c).write((short) 0x01);
        SYSALS__START.on(i2c).write((short) 0x01);

        INTERLEAVED_MODE__ENABLE.on(i2c).write((short) 0);
    }

    @Override
    public boolean dataReadyRange() {
        return (RESULT__INTERRUPT_STATUS_GPIO.on(i2c).read() & 0x04) != 0;
    }

    @Override
    public short readRangeContinuous() {
        checkState(dataReadyRange(), "Data not ready, check dataReadyRange()");
        short range = RESULT__RANGE_VAL.on(i2c).read();
        SYSTEM__INTERRUPT_CLEAR.on(i2c).write((short) 0x01);

        return range;
    }

    @Override
    public boolean dataReadyAmbient() {
        return (RESULT__INTERRUPT_STATUS_GPIO.on(i2c).read() & 0x20) != 0;
    }

    @Override
    public int readAmbientContinuous() {
        checkState(dataReadyAmbient(), "Data not ready, check dataReadyAmbient()");

        int ambient = RESULT__ALS_VAL.on(i2c).read16Bit();
        SYSTEM__INTERRUPT_CLEAR.on(i2c).write((short) 0x02);

        return ambient;
    }

    @Override
    public void setTimeout(long timeout, TimeUnit unit) {
        checkArgument(timeout >= 0, "Timeout must be positive");
        Objects.requireNonNull(unit, "unit");
        this.timeout = unit.toNanos(timeout);
    }

    @Override
    public long getTimeout(TimeUnit unit) {
        return unit.convert(timeout, TimeUnit.NANOSECONDS);
    }

    @Override
    public boolean timeoutOccurred() {
        boolean hasTimedOut = didTimeout;
        didTimeout = false;
        return hasTimedOut;
    }

// internal functions:

    void startTimeout() {
        timeoutStart = System.nanoTime();
    }

    boolean currentlyTimedOut() {
        if (timeout <= 0) {
            return false;
        }

        return timeoutStart + timeout < System.nanoTime();
    }

    void setTimeoutFlag() {
        didTimeout = true;
    }
}
