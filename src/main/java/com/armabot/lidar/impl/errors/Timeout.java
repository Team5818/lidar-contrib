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

public class Timeout extends ErrorBase<Timeout.Data> {

    public static Timeout waitingFor(String waitingFor) {
        return new Timeout(
                new AutoValue_Timeout_Data(waitingFor));
    }

    @AutoValue
    public static abstract class Data {

        public abstract String waitingFor();

        @Override
        public final String toString() {
            return "Timed out waiting for " + waitingFor();
        }
    }

    private Timeout(Data data) {
        super("timeout", data);
    }

}
