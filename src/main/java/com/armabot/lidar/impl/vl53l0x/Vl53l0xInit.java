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

package com.armabot.lidar.impl.vl53l0x;

import com.armabot.lidar.arcompat.PololuI2c;
import com.armabot.lidar.arcompat.Register;

import java.util.Optional;

import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.DYNAMIC_SPAD_REF_EN_START_OFFSET;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.GLOBAL_CONFIG_REF_EN_START_SELECT;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.GPIO_HV_MUX_ACTIVE_HIGH;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.MSRC_CONFIG_CONTROL;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_INTERRUPT_CLEAR;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_INTERRUPT_CONFIG_GPIO;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_SEQUENCE_CONFIG;
import static com.armabot.lidar.impl.vl53l0x.Vl53l0xReg.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV;

class Vl53l0xInit {
    private final Vl53l0xI2c target;
    private final PololuI2c i2c;

    Vl53l0xInit(Vl53l0xI2c target) {
        this.target = target;
        this.i2c = target.getI2c();
    }

    boolean initialize() {
        Register.Bound hv = VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV.on(i2c);
        hv.write((short) (hv.read() | 0x01));

        // "Set I2C standard mode"
        i2c.writeReg((short) 0x88, (short) 0x00);

        i2c.writeReg((short) 0x80, (short) 0x01);
        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x00, (short) 0x00);
        target.stopVariable = i2c.readReg((short) 0x91);
        i2c.writeReg((short) 0x00, (short) 0x01);
        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x80, (short) 0x00);

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        Register.Bound msrcConfigControl = MSRC_CONFIG_CONTROL.on(i2c);
        msrcConfigControl.write((short) (msrcConfigControl.read() | 0x12));

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        target.setSignalRateLimit(0.25f);

        SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0xFF);

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        Optional<SpadInfo> spadInfoOptional = target.getSpadInfo();
        if (!spadInfoOptional.isPresent()) {
            return false;
        }
        SpadInfo spadInfo = spadInfoOptional.get();

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        byte[] ref_spad_map = new byte[6];
        GLOBAL_CONFIG_SPAD_ENABLES_REF_0.on(i2c).readMulti(ref_spad_map);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        i2c.writeReg((short) 0xFF, (short) 0x01);
        DYNAMIC_SPAD_REF_EN_START_OFFSET.on(i2c).write((short) 0x00);
        DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD.on(i2c).write((short) 0x2C);
        i2c.writeReg((short) 0xFF, (short) 0x00);
        GLOBAL_CONFIG_REF_EN_START_SELECT.on(i2c).write((short) 0xB4);

        int first_spad_to_enable = spadInfo.isAperture ? 12 : 0; // 12 is the first aperture spad
        int spads_enabled = 0;

        for (short i = 0; i < 48; i++) {
            if (i < first_spad_to_enable || spads_enabled == spadInfo.count) {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            } else if (((ref_spad_map[i / 8] >> (i % 8)) & 0x1) != 0) {
                spads_enabled++;
            }
        }

        GLOBAL_CONFIG_SPAD_ENABLES_REF_0.on(i2c).writeMulti(ref_spad_map);

        // -- VL53L0X_set_reference_spads() end

        loadTuningSettings();

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        SYSTEM_INTERRUPT_CONFIG_GPIO.on(i2c).write((short) 0x04);
        Register.Bound gpioHv = GPIO_HV_MUX_ACTIVE_HIGH.on(i2c);
        gpioHv.write((short) (gpioHv.read() & ~0x10)); // active low
        SYSTEM_INTERRUPT_CLEAR.on(i2c).write((short) 0x01);

        // -- VL53L0X_SetGpioConfig() end

        target.measurementTimingBudgetMicrosec = target.getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        target.setMeasurementTimingBudget(target.measurementTimingBudgetMicrosec);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0x01);
        if (!target.performSingleRefCalibration((short) 0x40)) {
            return false;
        }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0x02);
        if (!target.performSingleRefCalibration((short) 0x00)) {
            return false;
        }

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0xE8);

        // VL53L0X_PerformRefCalibration() end

        return true;
    }

    private void loadTuningSettings() {
        // DefaultTuningSettings from vl53l0x_tuning.h

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x00, (short) 0x00);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x09, (short) 0x00);
        i2c.writeReg((short) 0x10, (short) 0x00);
        i2c.writeReg((short) 0x11, (short) 0x00);

        i2c.writeReg((short) 0x24, (short) 0x01);
        i2c.writeReg((short) 0x25, (short) 0xFF);
        i2c.writeReg((short) 0x75, (short) 0x00);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x4E, (short) 0x2C);
        i2c.writeReg((short) 0x48, (short) 0x00);
        i2c.writeReg((short) 0x30, (short) 0x20);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x30, (short) 0x09);
        i2c.writeReg((short) 0x54, (short) 0x00);
        i2c.writeReg((short) 0x31, (short) 0x04);
        i2c.writeReg((short) 0x32, (short) 0x03);
        i2c.writeReg((short) 0x40, (short) 0x83);
        i2c.writeReg((short) 0x46, (short) 0x25);
        i2c.writeReg((short) 0x60, (short) 0x00);
        i2c.writeReg((short) 0x27, (short) 0x00);
        i2c.writeReg((short) 0x50, (short) 0x06);
        i2c.writeReg((short) 0x51, (short) 0x00);
        i2c.writeReg((short) 0x52, (short) 0x96);
        i2c.writeReg((short) 0x56, (short) 0x08);
        i2c.writeReg((short) 0x57, (short) 0x30);
        i2c.writeReg((short) 0x61, (short) 0x00);
        i2c.writeReg((short) 0x62, (short) 0x00);
        i2c.writeReg((short) 0x64, (short) 0x00);
        i2c.writeReg((short) 0x65, (short) 0x00);
        i2c.writeReg((short) 0x66, (short) 0xA0);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x22, (short) 0x32);
        i2c.writeReg((short) 0x47, (short) 0x14);
        i2c.writeReg((short) 0x49, (short) 0xFF);
        i2c.writeReg((short) 0x4A, (short) 0x00);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x7A, (short) 0x0A);
        i2c.writeReg((short) 0x7B, (short) 0x00);
        i2c.writeReg((short) 0x78, (short) 0x21);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x23, (short) 0x34);
        i2c.writeReg((short) 0x42, (short) 0x00);
        i2c.writeReg((short) 0x44, (short) 0xFF);
        i2c.writeReg((short) 0x45, (short) 0x26);
        i2c.writeReg((short) 0x46, (short) 0x05);
        i2c.writeReg((short) 0x40, (short) 0x40);
        i2c.writeReg((short) 0x0E, (short) 0x06);
        i2c.writeReg((short) 0x20, (short) 0x1A);
        i2c.writeReg((short) 0x43, (short) 0x40);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x34, (short) 0x03);
        i2c.writeReg((short) 0x35, (short) 0x44);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x31, (short) 0x04);
        i2c.writeReg((short) 0x4B, (short) 0x09);
        i2c.writeReg((short) 0x4C, (short) 0x05);
        i2c.writeReg((short) 0x4D, (short) 0x04);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x44, (short) 0x00);
        i2c.writeReg((short) 0x45, (short) 0x20);
        i2c.writeReg((short) 0x47, (short) 0x08);
        i2c.writeReg((short) 0x48, (short) 0x28);
        i2c.writeReg((short) 0x67, (short) 0x00);
        i2c.writeReg((short) 0x70, (short) 0x04);
        i2c.writeReg((short) 0x71, (short) 0x01);
        i2c.writeReg((short) 0x72, (short) 0xFE);
        i2c.writeReg((short) 0x76, (short) 0x00);
        i2c.writeReg((short) 0x77, (short) 0x00);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x0D, (short) 0x01);

        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x80, (short) 0x01);
        i2c.writeReg((short) 0x01, (short) 0xF8);

        i2c.writeReg((short) 0xFF, (short) 0x01);
        i2c.writeReg((short) 0x8E, (short) 0x01);
        i2c.writeReg((short) 0x00, (short) 0x01);
        i2c.writeReg((short) 0xFF, (short) 0x00);
        i2c.writeReg((short) 0x80, (short) 0x00);
    }
}
