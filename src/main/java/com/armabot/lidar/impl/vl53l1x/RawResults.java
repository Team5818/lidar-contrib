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

import com.google.auto.value.AutoValue;

@AutoValue
abstract class RawResults {

    static Builder builder() {
        return new AutoValue_RawResults.Builder();
    }

    @AutoValue.Builder
    interface Builder {

        Builder rangeStatus(short rangeStatus);

        Builder streamCount(short streamCount);

        Builder dssActualEffectiveSpadsSd0(int dssActualEffectiveSpadsSd0);

        Builder ambientCountRateMcpsSd0(int ambientCountRateMcpsSd0);

        Builder finalCrosstalkCorrectRangeMmSd0(int finalCrosstalkCorrectRangeMmSd0);

        Builder peakSignalCountRateCrosstalkCorrectedMcpsSd0(int peakSignalCountRateCrosstalkCorrectedMcpsSd0);

        RawResults build();
    }

    RawResults() {
    }

    abstract short rangeStatus();

    abstract short streamCount();

    abstract int dssActualEffectiveSpadsSd0();

    abstract int ambientCountRateMcpsSd0();

    abstract int finalCrosstalkCorrectRangeMmSd0();

    abstract int peakSignalCountRateCrosstalkCorrectedMcpsSd0();

}
