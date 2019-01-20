package com.armabot.lidar.impl.vl53l0x.errors;

import com.armabot.lidar.impl.ErrorBase;
import com.google.auto.value.AutoValue;

public class NoSpadInfo extends ErrorBase<NoSpadInfo.Data> {

    public static NoSpadInfo getInstance() {
        return new NoSpadInfo(
                new AutoValue_NoSpadInfo_Data());
    }

    @AutoValue
    public static abstract class Data {

        @Override
        public final String toString() {
            return "No spad info";
        }
    }

    private NoSpadInfo(Data data) {
        super("no.spad.info", data);
    }

}
