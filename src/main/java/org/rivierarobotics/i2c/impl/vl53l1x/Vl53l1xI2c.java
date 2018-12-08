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

package org.rivierarobotics.i2c.impl.vl53l1x;

import edu.wpi.first.wpilibj.I2C.Port;
import org.rivierarobotics.i2c.api.Vl53lx;
import org.rivierarobotics.i2c.arcompat.PololuI2c;
import org.rivierarobotics.i2c.arcompat.Register;
import org.rivierarobotics.i2c.arcompat.Wire;
import org.rivierarobotics.i2c.util.Preconditions;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

import static org.rivierarobotics.i2c.impl.vl53l1x.Calculations.calcMacroPeriod;
import static org.rivierarobotics.i2c.impl.vl53l1x.Calculations.decodeTimeout;
import static org.rivierarobotics.i2c.impl.vl53l1x.Calculations.encodeTimeout;
import static org.rivierarobotics.i2c.impl.vl53l1x.Calculations.timeoutMclksToMicroseconds;
import static org.rivierarobotics.i2c.impl.vl53l1x.Calculations.timeoutMicrosecondsToMclks;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.CAL_CONFIG__VCSEL_START;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.GPIO__TIO_HV_STATUS;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.I2C_SLAVE__DEVICE_ADDRESS;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.MM_CONFIG__TIMEOUT_MACROP_A;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.MM_CONFIG__TIMEOUT_MACROP_B;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.PHASECAL_CONFIG__OVERRIDE;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.PHASECAL_CONFIG__TIMEOUT_MACROP;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.PHASECAL_RESULT__VCSEL_START;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RANGE_CONFIG__TIMEOUT_MACROP_A;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RANGE_CONFIG__TIMEOUT_MACROP_B;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RANGE_CONFIG__VALID_PHASE_HIGH;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_A;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RANGE_CONFIG__VCSEL_PERIOD_B;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.RESULT__RANGE_STATUS;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SD_CONFIG__INITIAL_PHASE_SD0;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SD_CONFIG__INITIAL_PHASE_SD1;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SD_CONFIG__WOI_SD0;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SD_CONFIG__WOI_SD1;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SYSTEM__INTERMEASUREMENT_PERIOD;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SYSTEM__INTERRUPT_CLEAR;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.SYSTEM__MODE_START;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.VHV_CONFIG__INIT;
import static org.rivierarobotics.i2c.impl.vl53l1x.Vl53l1xReg.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND;

/**
 * A near-direct port of the
 * <a href="https://github.com/pololu/vl53l1x-arduino">vl53l1x-arduino</a>
 * library.
 */
public class Vl53l1xI2c implements Vl53lx {

    private static final short TIMING_GUARD = 4528;
    static final short TARGET_RATE = 0x0A00;

    private final PololuI2c i2c;

    private short fastOscFreq;
    private short oscCalibrateVal;
    private DistanceMode distanceMode = DistanceMode.UNKNOWN;
    private boolean calibrated;
    private long timeout;
    private long timeoutStart;
    private boolean didTimeout;
    private byte savedVhvInit;
    private byte savedVhvTimeout;
    private RawResults results;

    public Vl53l1xI2c(Port port) {
        this.i2c = PololuI2c.create(port);
        setAddress(Vl53lx.DEFAULT_ADDRESS);
    }

    @Override
    public byte getAddress() {
        return i2c.getAddress();
    }

    @Override
    public void setAddress(byte address) {
        I2C_SLAVE__DEVICE_ADDRESS.on(i2c).write((byte) (address & 0x7F));
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

    void fastOscFreq(short fastOscFreq) {
        this.fastOscFreq = fastOscFreq;
    }

    void oscCalibrateVal(short oscCalibrateVal) {
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
        RANGE_CONFIG__VCSEL_PERIOD_A.on(i2c).write(vcselPeriodA);
        RANGE_CONFIG__VCSEL_PERIOD_B.on(i2c).write(vcselPeriodB);
        RANGE_CONFIG__VALID_PHASE_HIGH.on(i2c).write(validPhaseHigh);

        SD_CONFIG__WOI_SD0.on(i2c).write(woiSd0);
        SD_CONFIG__WOI_SD1.on(i2c).write(woiSd1);
        SD_CONFIG__INITIAL_PHASE_SD0.on(i2c).write(initialPhaseSd0);
        SD_CONFIG__INITIAL_PHASE_SD1.on(i2c).write(initialPhaseSd1);

        setMeasurementTimingBudget(budget);

        distanceMode = mode;
        return true;
    }

    private int currentMacroPeriodA() {
        return calcMacroPeriod(fastOscFreq, RANGE_CONFIG__VCSEL_PERIOD_A.on(i2c).read());
    }

    private int currentMacroPeriodB() {
        return calcMacroPeriod(fastOscFreq, RANGE_CONFIG__VCSEL_PERIOD_B.on(i2c).read());
    }

    @Override
    public int getMeasurementTimingBudget() {
        int macroPeriodMicrosec = currentMacroPeriodA();

        int rangeConfigTimeoutMicrosec = timeoutMclksToMicroseconds(
                decodeTimeout(RANGE_CONFIG__TIMEOUT_MACROP_A.on(i2c).read16Bit()),
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

        int phasecalTimeoutMclks = timeoutMclksToMicroseconds(100, macroPeriodMicrosec);
        if (phasecalTimeoutMclks > 0xFF) {
            phasecalTimeoutMclks = 0xFF;
        }

        PHASECAL_CONFIG__TIMEOUT_MACROP.on(i2c).write((byte) phasecalTimeoutMclks);

        MM_CONFIG__TIMEOUT_MACROP_A.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(1, macroPeriodMicrosec)
        ));

        RANGE_CONFIG__TIMEOUT_MACROP_A.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(rangeConfigTimeoutMicrosec, macroPeriodMicrosec)
        ));

        macroPeriodMicrosec = currentMacroPeriodB();

        MM_CONFIG__TIMEOUT_MACROP_B.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(1, macroPeriodMicrosec)
        ));

        RANGE_CONFIG__TIMEOUT_MACROP_B.on(i2c).write16Bit(encodeTimeout(
                timeoutMicrosecondsToMclks(rangeConfigTimeoutMicrosec, macroPeriodMicrosec)
        ));

        return true;
    }

    @Override
    public void startContinuous(int periodMillis) {
        SYSTEM__INTERMEASUREMENT_PERIOD.on(i2c).write32Bit(periodMillis * oscCalibrateVal);
        SYSTEM__INTERRUPT_CLEAR.on(i2c).write((byte) 0x01);
        SYSTEM__MODE_START.on(i2c).write((byte) 0x40);
    }

    @Override
    public void stopContinuous() {
        SYSTEM__MODE_START.on(i2c).write((byte) 0x80);


        calibrated = false;

        // "restore vhv configs"
        if (savedVhvInit != 0) {
            VHV_CONFIG__INIT.on(i2c).write(savedVhvInit);
        }
        if (savedVhvTimeout != 0) {
            VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND.on(i2c).write(savedVhvTimeout);
        }

        // "remove phasecal override"
        PHASECAL_CONFIG__OVERRIDE.on(i2c).write((byte) 0x00);
    }

    @Override
    public short read() {

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

        SYSTEM__INTERRUPT_CLEAR.on(i2c).write((byte) 1);

        // just directly calculate for now, no getRangeData
        short range = results.finalCrosstalkCorrectRangeMmSd0();

        return (short) ((Short.toUnsignedInt(range) * 2011 + 0x0400) / 0x0800);
    }

    private void readResults() {
        Wire wire = i2c.getWire();

        i2c.beginTransmission();
        i2c.askForRegValue(RESULT__RANGE_STATUS.address());
        i2c.endTransmission();

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
        Register.Bound vhvConfigInit = VHV_CONFIG__INIT.on(i2c);
        Register.Bound vhvConfigTimeout = VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND.on(i2c);

        savedVhvInit = vhvConfigInit.read();
        savedVhvTimeout = vhvConfigTimeout.read();

        vhvConfigInit.write((byte) (savedVhvInit & 0x7F));
        vhvConfigTimeout.write((byte) ((savedVhvTimeout & 0x03) + (3 << 2)));

        PHASECAL_CONFIG__OVERRIDE.on(i2c).write((byte) 0x01);
        CAL_CONFIG__VCSEL_START.on(i2c).write(
                PHASECAL_RESULT__VCSEL_START.on(i2c).read()
        );
    }

    private void updateDss() {
        short spadCount = results.dssActualEffectiveSpadsSd0();

        if (spadCount != 0) {
            // "Calc total rate per spad"

            int peakSignal = Short.toUnsignedInt(results.peakSignalCountRateCrosstalkCorrectedMcpsSd0());
            int ambientCount = Short.toUnsignedInt(results.ambientCountRateMcpsSd0());
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
                DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT.on(i2c).write16Bit((short) requiredSpads);
                return;
            }
        }

        DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT.on(i2c).write16Bit((short) 0x8000);
    }

    @Override
    public boolean dataReady() {
        return (GPIO__TIO_HV_STATUS.on(i2c).read() & 0x01) == 0;
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
