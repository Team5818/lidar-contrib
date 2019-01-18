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

import com.armabot.lidar.arcompat.Register;
import com.armabot.lidar.util.Preconditions;

/**
 * Taken from the Arduino library code.
 */
public enum Vl53l0xReg implements Register {
    SYSRANGE_START(0x00),

    SYSTEM_THRESH_HIGH(0x0C),
    SYSTEM_THRESH_LOW(0x0E),

    SYSTEM_SEQUENCE_CONFIG(0x01),
    SYSTEM_RANGE_CONFIG(0x09),
    SYSTEM_INTERMEASUREMENT_PERIOD(0x04),

    SYSTEM_INTERRUPT_CONFIG_GPIO(0x0A),

    GPIO_HV_MUX_ACTIVE_HIGH(0x84),

    SYSTEM_INTERRUPT_CLEAR(0x0B),

    RESULT_INTERRUPT_STATUS(0x13),
    RESULT_RANGE_STATUS(0x14),

    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN(0xBC),
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN(0xC0),
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF(0xD0),
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF(0xD4),
    RESULT_PEAK_SIGNAL_RATE_REF(0xB6),

    ALGO_PART_TO_PART_RANGE_OFFSET_MM(0x28),

    I2C_SLAVE_DEVICE_ADDRESS(0x8A),

    MSRC_CONFIG_CONTROL(0x60),

    PRE_RANGE_CONFIG_MIN_SNR(0x27),
    PRE_RANGE_CONFIG_VALID_PHASE_LOW(0x56),
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH(0x57),
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT(0x64),

    FINAL_RANGE_CONFIG_MIN_SNR(0x67),
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW(0x47),
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH(0x48),
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT(0x44),

    PRE_RANGE_CONFIG_SIGMA_THRESH_HI(0x61),
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO(0x62),

    PRE_RANGE_CONFIG_VCSEL_PERIOD(0x50),
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI(0x51),
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO(0x52),

    SYSTEM_HISTOGRAM_BIN(0x81),
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT(0x33),
    HISTOGRAM_CONFIG_READOUT_CTRL(0x55),

    FINAL_RANGE_CONFIG_VCSEL_PERIOD(0x70),
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI(0x71),
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO(0x72),
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS(0x20),

    MSRC_CONFIG_TIMEOUT_MACROP(0x46),

    SOFT_RESET_GO2_SOFT_RESET_N(0xBF),
    IDENTIFICATION_MODEL_ID(0xC0),
    IDENTIFICATION_REVISION_ID(0xC2),

    OSC_CALIBRATE_VAL(0xF8),

    GLOBAL_CONFIG_VCSEL_WIDTH(0x32),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0(0xB0),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1(0xB1),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2(0xB2),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3(0xB3),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4(0xB4),
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5(0xB5),

    GLOBAL_CONFIG_REF_EN_START_SELECT(0xB6),
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD(0x4E),
    DYNAMIC_SPAD_REF_EN_START_OFFSET(0x4F),
    POWER_MANAGEMENT_GO1_POWER_FORCE(0x80),

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV(0x89),

    ALGO_PHASECAL_LIM(0x30),
    ALGO_PHASECAL_CONFIG_TIMEOUT(0x30),
    ;

    private final short address;

    Vl53l0xReg(int address) {
        Preconditions.checkArgument(0 <= address && address <= Short.MAX_VALUE, "address must be a positive short");
        this.address = (short) address;
    }

    public short address() {
        return address;
    }
}
