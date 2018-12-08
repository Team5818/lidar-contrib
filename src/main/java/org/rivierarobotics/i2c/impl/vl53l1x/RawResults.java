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

import com.google.auto.value.AutoValue;

@AutoValue
abstract class RawResults {

    static Builder builder() {
        return new AutoValue_RawResults.Builder();
    }

    @AutoValue.Builder
    interface Builder {

        Builder rangeStatus(byte rangeStatus);

        Builder streamCount(byte streamCount);

        Builder dssActualEffectiveSpadsSd0(short dssActualEffectiveSpadsSd0);

        Builder ambientCountRateMcpsSd0(short ambientCountRateMcpsSd0);

        Builder finalCrosstalkCorrectRangeMmSd0(short finalCrosstalkCorrectRangeMmSd0);

        Builder peakSignalCountRateCrosstalkCorrectedMcpsSd0(short peakSignalCountRateCrosstalkCorrectedMcpsSd0);

        RawResults build();
    }

    RawResults() {
    }

    abstract byte rangeStatus();

    abstract byte streamCount();

    abstract short dssActualEffectiveSpadsSd0();

    abstract short ambientCountRateMcpsSd0();

    abstract short finalCrosstalkCorrectRangeMmSd0();

    abstract short peakSignalCountRateCrosstalkCorrectedMcpsSd0();

}
