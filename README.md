lidar-contrib
=============

![Maven metadata URL](https://img.shields.io/maven-metadata/v?metadataUrl=https%3A%2F%2Fmaven.octyl.net%2Frepository%2Farmabot-releases%2Fcom%2Farmabot%2Flidar-contrib%2Fmaven-metadata.xml&style=flat-square)

FRC compatibility layer for Pololu sensors.

Available from the https://maven.octyl.net/repository/armabot-releases/ repository
(or `-snapshots` for SNAPSHOT versions) with the following Gradle declarations:

```kotlin
repositories {
    maven {
        name = "ARMABOT"
        uri = "https://maven.octyl.net/repository/armabot-releases/"
    }
}

dependencies {
    // Substitute a proper version, see badge above
    implementation("com.armabot:lidar-contrib:VERSION")
}
```


Based heavily on https://github.com/pololu/vl53l1x-arduino and related
projects. See them for implementation details.

In general, functions are either documented or have direct counterparts in the Arduino projects.
Using the API will look something like the following sample code, taken from a `Robot` class:
```java
public class Robot extends TimedRobot {
    private Vl6180x l0xSensor;
    private final NetworkTableEntry l0x = Shuffleboard.getTab("Sensors")
        .add("VL618X", "No data").getEntry();

    @Override
    public void robotInit() {
        l0xSensor = new Vl6180xI2c(RoboRioPort.ONBOARD);
        // You'll need to make sure this is the right address, see the sensor's manual for how to set it
        l0xSensor.getI2c().setAddress((byte) 0x29);
        l0xSensor.setTimeout(10, TimeUnit.SECONDS);
        while (true) {
            var error = l0xSensor.initialize();
            error.ifPresent(err ->
                DriverStation.reportError("Unable to initialize VL53L0X: " + err, false)
            );
            if (error.isEmpty()) {
                break;
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        }
        l0xSensor.configureDefault();
        l0xSensor.startRangeContinuous(100);
    }

    @Override
    public void disabledPeriodic() {
        if (l0xSensor != null && l0xSensor.dataReadyRange()) {
            var output = l0xSensor.readRangeContinuousMillimeters() + (l0xSensor.timeoutOccurred() ? " (TIMEOUT!)" : "");
            l0x.setString(output);
        }
    }
}
```
