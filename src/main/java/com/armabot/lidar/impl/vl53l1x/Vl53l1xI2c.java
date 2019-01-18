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

package com.armabot.lidar.impl.vl53l1x;

import com.armabot.lidar.api.Vl53l1x;
import com.armabot.lidar.arcompat.PololuI2c;
import com.armabot.lidar.arcompat.Port;
import com.armabot.lidar.arcompat.Register;
import com.armabot.lidar.arcompat.Wire;
import com.armabot.lidar.util.Preconditions;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

import static com.armabot.lidar.impl.vl53l1x.Calculations.calcMacroPeriod;
import static com.armabot.lidar.impl.vl53l1x.Calculations.decodeTimeout;
import static com.armabot.lidar.impl.vl53l1x.Calculations.encodeTimeout;
import static com.armabot.lidar.impl.vl53l1x.Calculations.timeoutMclksToMicroseconds;
import static com.armabot.lidar.impl.vl53l1x.Calculations.timeoutMicrosecondsToMclks;

/**
 * A near-direct port of the
 * <a href="https://github.com/pololu/vl53l1x-arduino">vl53l1x-arduino</a>
 * library.
 */
// The original library is licensed under the terms in LICENSE-vl53l1x.txt
public class Vl53l1xI2c implements Vl53l1x {

    private static final int TIMING_GUARD = 4528;
    static final int TARGET_RATE = 0x0A00;

    private final PololuI2c i2c;

    private int fastOscFreq;
    private int oscCalibrateVal;
    private DistanceMode distanceMode = DistanceMode.UNKNOWN;
    private boolean calibrated;
    private long timeout;
    private long timeoutStart;
    private boolean didTimeout;
    private short savedVhvInit;
    private short savedVhvTimeout;
    private RawResults results;

    public Vl53l1xI2c(Port port) {
        this.i2c = PololuI2c.create(port, PololuI2c.Addressing.SIXTEEN_BIT);
        i2c.setAddress(Vl53l1x.DEFAULT_ADDRESS);
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
        Vl53l1xReg.I2C_SLAVE__DEVICE_ADDRESS.on(i2c).write((byte) (address & 0x7F));
        i2c.setAddress(address);
    }

    @Override
    public boolean initialize() {
        return new Vl53l1xInit(this).initialize();
    }

    @Override
    public PololuI2c getI2c() {
        return i2c;
    }

    void fastOscFreq(int fastOscFreq) {
        this.fastOscFreq = fastOscFreq;
    }

    void oscCalibrateVal(int oscCalibrateVal) {
        this.oscCalibrateVal = oscCalibrateVal;
    }

    @Override
    public DistanceMode getDistanceMode() {
        return distanceMode;
    }

    @Override
    public boolean setDistanceMode(DistanceMode mode) {
        int budget = getMeasurementTimingBudget();

        byte vcselPeriodA;
        byte vcselPeriodB;
        byte validPhaseHigh;
        byte woiSd0;
        byte woiSd1;
        byte initialPhaseSd0;
        byte initialPhaseSd1;
        switch (mode) {
            case SHORT:
                vcselPeriodA = 0x07;
                vcselPeriodB = 0x05;
                validPhaseHigh = 0x38;

                woiSd0 = 0x07;
                woiSd1 = 0x05;
                initialPhaseSd0 = 6;
                initialPhaseSd1 = 6;
                break;
            case MEDIUM:
                vcselPeriodA = 0x0B;
                vcselPeriodB = 0x09;
                validPhaseHigh = 0x78;

                woiSd0 = 0x0B;
                woiSd1 = 0x09;
                initialPhaseSd0 = 10;
                initialPhaseSd1 = 10;
                break;
            case LONG:
                vcselPeriodA = 0x0F;
                vcselPeriodB = 0x0D;
                validPhaseHigh = (byte) 0xB8;

                woiSd0 = 0x0F;
                woiSd1 = 0x0D;
                initialPhaseSd0 = 14;
                initialPhaseSd1 = 14;
                break;
            case UNKNOWN:
            default:
                return false;
        }
        Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_A.on(i2c).write(vcselPeriodA);
        Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_B.on(i2c).write(vcselPeriodB);
        Vl53l1xReg.RANGE_CONFIG__VALID_PHASE_HIGH.on(i2c).write(validPhaseHigh);

        Vl53l1xReg.SD_CONFIG__WOI_SD0.on(i2c).write(woiSd0);
        Vl53l1xReg.SD_CONFIG__WOI_SD1.on(i2c).write(woiSd1);
        Vl53l1xReg.SD_CONFIG__INITIAL_PHASE_SD0.on(i2c).write(initialPhaseSd0);
        Vl53l1xReg.SD_CONFIG__INITIAL_PHASE_SD1.on(i2c).write(initialPhaseSd1);

        setMeasurementTimingBudget(budget);

        distanceMode = mode;
        return true;
    }

    private int currentMacroPeriodA() {
        return calcMacroPeriod(fastOscFreq, Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_A.on(i2c).read());
    }

    private int currentMacroPeriodB() {
        return calcMacroPeriod(fastOscFreq, Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_B.on(i2c).read());
    }

    @Override
    public int getMeasurementTimingBudget() {
        int macroPeriodMicrosec = currentMacroPeriodA();

        int rangeConfigTimeoutMicrosec = timeoutMclksToMicroseconds(
                decodeTimeout(Vl53l1xReg.RANGE_CONFIG__TIMEOUT_MACROP_A.on(i2c).read16Bit()),
                macroPeriodMicrosec);

        return 2 * rangeConfigTimeoutMicrosec + TIMING_GUARD;
    }

    @Override
    public boolean setMeasurementTimingBudget(int budgetMicro) {
        if (budgetMicro < TIMING_GUARD) {
            return false;
        }

        int rangeConfigTimeoutMicrosec = (budgetMicro - TIMING_GUARD) / 2;
        if (rangeConfigTimeoutMicrosec > 550_000) {
            return false;
        }

        int macroPeriodMicrosec = currentMacroPeriodA();

        int phasecalTimeoutMclks = timeoutMicrosecondsToMclks(1000, macroPeriodMicrosec);
        if (phasecalTimeoutMclks > 0xFF) {
            phasecalTimeoutMclks = 0xFF;
        }

        Vl53l1xReg.PHASECAL_CONFIG__TIMEOUT_MACROP.on(i2c).write((short) phasecalTimeoutMclks);

        int timeoutMclks = timeoutMicrosecondsToMclks(1, macroPeriodMicrosec);
        int value = encodeTimeout(
                timeoutMclks
        );
        Vl53l1xReg.MM_CONFIG__TIMEOUT_MACROP_A.on(i2c).write16Bit(value);

        Vl53l1xReg.RANGE_CONFIG__TIMEOUT_MACROP_A.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(rangeConfigTimeoutMicrosec, macroPeriodMicrosec)
        ));

        macroPeriodMicrosec = currentMacroPeriodB();

        Vl53l1xReg.MM_CONFIG__TIMEOUT_MACROP_B.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(1, macroPeriodMicrosec)
        ));

        Vl53l1xReg.RANGE_CONFIG__TIMEOUT_MACROP_B.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(rangeConfigTimeoutMicrosec, macroPeriodMicrosec)
        ));

        return true;
    }

    @Override
    public void startContinuous(int periodMillis) {
        Vl53l1xReg.SYSTEM__INTERMEASUREMENT_PERIOD.on(i2c).write32Bit(periodMillis * oscCalibrateVal);
        Vl53l1xReg.SYSTEM__INTERRUPT_CLEAR.on(i2c).write((byte) 0x01);
        Vl53l1xReg.SYSTEM__MODE_START.on(i2c).write((byte) 0x40);
    }

    @Override
    public void stopContinuous() {
        Vl53l1xReg.SYSTEM__MODE_START.on(i2c).write((byte) 0x80);


        calibrated = false;

        // "restore vhv configs"
        if (savedVhvInit != 0) {
            Vl53l1xReg.VHV_CONFIG__INIT.on(i2c).write(savedVhvInit);
        }
        if (savedVhvTimeout != 0) {
            Vl53l1xReg.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND.on(i2c).write(savedVhvTimeout);
        }

        // "remove phasecal override"
        Vl53l1xReg.PHASECAL_CONFIG__OVERRIDE.on(i2c).write((byte) 0x00);
    }

    @Override
    public int read() {

        startTimeout();
        while (!dataReady()) {
            if (currentlyTimedOut()) {
                setTimeoutFlag();
                return 0;
            }
        }

        readResults();

        if (!calibrated) {
            calibrate();
            calibrated = true;
        }

        updateDss();

        Vl53l1xReg.SYSTEM__INTERRUPT_CLEAR.on(i2c).write((byte) 1);

        // just directly calculate for now, no getRangeData
        int range = results.finalCrosstalkCorrectRangeMmSd0();

        return (range * 2011 + 0x0400) / 0x0800;
    }

    private void readResults() {
        Wire wire = i2c.getWire();

        i2c.askForRegValue(Vl53l1xReg.RESULT__RANGE_STATUS.address());

        i2c.request(17);

        RawResults.Builder results = RawResults.builder();

        results.rangeStatus(wire.read());

        // report_status unused:
        wire.read();

        results.streamCount(wire.read());

        results.dssActualEffectiveSpadsSd0(wire.readShort());

        // peak_signal_count_rate_mcps_sd0: not used
        wire.readShort();

        results.ambientCountRateMcpsSd0(wire.readShort());

        // sigma_sd0: not used
        wire.readShort();

        // phase_sd0: not used
        wire.readShort();

        results.finalCrosstalkCorrectRangeMmSd0(wire.readShort());

        results.peakSignalCountRateCrosstalkCorrectedMcpsSd0(wire.readShort());

        this.results = results.build();
    }

    private void calibrate() {
        Register.Bound vhvConfigInit = Vl53l1xReg.VHV_CONFIG__INIT.on(i2c);
        Register.Bound vhvConfigTimeout = Vl53l1xReg.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND.on(i2c);

        savedVhvInit = vhvConfigInit.read();
        savedVhvTimeout = vhvConfigTimeout.read();

        vhvConfigInit.write((short) (savedVhvInit & 0x7F));
        vhvConfigTimeout.write((short) ((savedVhvTimeout & 0x03) + (3 << 2)));

        Vl53l1xReg.PHASECAL_CONFIG__OVERRIDE.on(i2c).write((byte) 0x01);
        Vl53l1xReg.CAL_CONFIG__VCSEL_START.on(i2c).write(
                Vl53l1xReg.PHASECAL_RESULT__VCSEL_START.on(i2c).read()
        );
    }

    private void updateDss() {
        int spadCount = results.dssActualEffectiveSpadsSd0();

        if (spadCount != 0) {
            // "Calc total rate per spad"

            int peakSignal = results.peakSignalCountRateCrosstalkCorrectedMcpsSd0();
            int ambientCount = results.ambientCountRateMcpsSd0();
            int totalRatePerSpad = peakSignal + ambientCount;

            // "clip to 16 bits"
            if (totalRatePerSpad > 0xFFFF) {
                totalRatePerSpad = 0xFFFF;
            }

            // "shift up to take advantage of 32 bits"
            totalRatePerSpad <<= 16;

            totalRatePerSpad /= spadCount;

            if (totalRatePerSpad != 0) {
                // "get the target rate and shift up by 16"
                int requiredSpads = (TARGET_RATE << 16) / totalRatePerSpad;

                // "clip to 16 bit"
                if (requiredSpads > 0xFFFF) {
                    requiredSpads = 0xFFFF;
                }

                // "override DSS config"
                Vl53l1xReg.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT.on(i2c).write16Bit(requiredSpads);
                return;
            }
        }

        Vl53l1xReg.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT.on(i2c).write16Bit(0x8000);
    }

    @Override
    public boolean dataReady() {
        return (Vl53l1xReg.GPIO__TIO_HV_STATUS.on(i2c).read() & 0x01) == 0;
    }

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

    @Override
    public void setTimeout(long timeout, TimeUnit unit) {
        Preconditions.checkArgument(timeout >= 0, "Timeout must be positive");
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
}
