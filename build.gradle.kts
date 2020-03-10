import com.techshroom.inciseblue.commonLib

plugins {
    id("net.researchgate.release") version "2.8.1"
    id("com.techshroom.incise-blue") version "0.5.7"
    id("net.ltgt.apt-idea") version "0.20"
    id("com.jfrog.bintray") version "1.8.4"
    `java-library`
    `maven-publish`
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
}

release {
    tagTemplate = "v\${version}"
}

java.withSourcesJar()
java.withJavadocJar()

publishing {
    publications {
        register<MavenPublication>("library") {
            from(components["java"])
        }
    }
}

bintray {
    user = System.getenv("BINTRAY_USER") ?: findProperty("bintray.user")?.toString()
    key = System.getenv("BINTRAY_KEY") ?: findProperty("bintray.password")?.toString()
    setPublications("library")
    with(pkg) {
        repo = "maven-release"
        name = project.name
        userOrg = "team5818"
        vcsUrl = "https://github.com/Team5818/lidar-contrib.git"
        publish = true
        setLicenses("GPL-3.0-or-later")
        with(version) {
            name = project.version.toString()
        }
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
