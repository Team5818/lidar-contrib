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
