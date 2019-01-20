import com.techshroom.inciseblue.commonLib
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.gradle.plugins.ide.idea.model.IdeaModel

plugins {
    id("net.researchgate.release") version "2.7.0"
    id("com.techshroom.incise-blue") version "0.2.2"
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
    from("LICENSE.txt")
}

inciseBlue {
    util {
        setJavaVersion("1.8")
    }
    license()
    ide()

    maven {
        projectDescription = "Pololu Device FRC Support"
        coords("Team5818", "pololu-frc-contrib")
        licenseName = "GPL"
    }
}

repositories {
    maven {
        name = "FRC"
        url = uri("http://first.wpi.edu/FRC/roborio/maven/release")
        metadataSources {
            mavenPom()
        }
    }
}

dependencies {
    commonLib("edu.wpi.first.hal", "hal", "2019.2.1") {
        api(lib("java"))
    }

    commonLib("com.google.auto.value", "auto-value", "1.6.2") {
        compileOnly(lib("annotations"))
        annotationProcessor(lib())
    }

    compileOnly("com.techshroom", "jsr305-plus", "0.0.1")

    testImplementation("junit:junit:4.12")
}
