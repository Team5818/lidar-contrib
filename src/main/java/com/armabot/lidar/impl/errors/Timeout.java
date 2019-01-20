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
