import com.techshroom.inciseblue.commonLib

plugins {
    id("net.researchgate.release") version "2.8.1"
    id("com.techshroom.incise-blue") version "0.5.7"
    id("net.ltgt.apt-idea") version "0.20"
    `java-library`
    `maven-publish`
}
// disable maven upload for now, don't have a good spot for this...
if (hasProperty("ossrhUsername")) {
    setProperty("ossrhUsername", "")
}

tasks.afterReleaseBuild {
    dependsOn("publishToMavenLocal")
}

tasks.processResources {
    from("LICENSE-vl53l1x.txt")
    from("LICENSE-vl53l0x.txt")
    from("LICENSE-vl6180x.txt")
    from("LICENSE.txt")
}

inciseBlue {
    util {
        setJavaVersion("11")
    }
    license()
    ide()

    nexus {
        projectDescription.set("Pololu Device FRC Support")
        coords("Team5818", "pololu-frc-contrib")
        licenseName.set("GPL")
    }
}

repositories {
    maven {
        name = "WPI"
        url = uri("https://frcmaven.wpi.edu/artifactory/release")
    }
}

dependencies {
    commonLib("edu.wpi.first.hal", "hal", "2020.3.2") {
        api(lib("java"))
    }

    commonLib("com.google.auto.value", "auto-value", "1.7") {
        compileOnly(lib("annotations"))
        annotationProcessor(lib())
    }

    compileOnly("com.techshroom", "jsr305-plus", "0.0.1")

    testImplementation("junit:junit:4.12")
}
