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

package com.armabot.lidar.impl.errors;

import com.armabot.lidar.impl.ErrorBase;
import com.google.auto.value.AutoValue;

public class IncorrectModelId extends ErrorBase<IncorrectModelId.Data> {

    public static IncorrectModelId of(String partId, int expectedId, int actualId) {
        return new IncorrectModelId(
                new AutoValue_IncorrectModelId_Data(partId, expectedId, actualId));
    }

    @AutoValue
    public static abstract class Data {

        public abstract String partId();

        public abstract int expectedId();

        public abstract int actualId();

        @Override
        public final String toString() {
            return String.format("%s expected to have model ID 0x%02x, got 0x%02x",
                    partId(),
                    expectedId(),
                    actualId());
        }
    }

    private IncorrectModelId(Data data) {
        super("incorrect.model.id", data);
    }

}
