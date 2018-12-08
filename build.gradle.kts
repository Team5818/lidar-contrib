import com.techshroom.inciseblue.commonLib
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.gradle.plugins.ide.idea.model.IdeaModel

plugins {
    id("net.researchgate.release") version "2.7.0"
    id("com.techshroom.incise-blue") version "0.2.2"
    id("net.ltgt.apt") version "0.19"
    `java-library`
    `maven-publish`
}
apply(plugin = "net.ltgt.apt-idea")

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
    commonLib("edu.wpi.first.wpilibj", "wpilibj", "2018.4.1") {
        api(lib("java"))
    }

    commonLib("com.google.auto.value", "auto-value", "1.6.2") {
        compileOnly(lib("annotations"))
        annotationProcessor(lib())
    }

    testImplementation("junit:junit:4.12")
}
