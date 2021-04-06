import org.cadixdev.gradle.licenser.LicenseExtension

plugins {
    `java-library`
    `maven-publish`
    id("net.researchgate.release") version "2.8.1"
    id("org.cadixdev.licenser") version "0.5.1"
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(11))
    }
    withSourcesJar()
    withJavadocJar()
}

repositories {
    mavenCentral()
    maven {
        name = "WPI"
        url = uri("https://frcmaven.wpi.edu/artifactory/release")
    }
}

dependencies {
    api("edu.wpi.first.hal:hal-java:2020.3.2")

    val autoValueVersion = "1.7.4"
    compileOnly("com.google.auto.value:auto-value-annotations:$autoValueVersion")
    annotationProcessor("com.google.auto.value:auto-value:$autoValueVersion")

    compileOnly("com.techshroom", "jsr305-plus", "0.0.1")
}

configure<LicenseExtension> {
    header = rootProject.file("HEADER.txt")
    (this as ExtensionAware).extra.apply {
        set("name", rootProject.name)
        for (key in listOf("organization", "url")) {
            set(key, rootProject.property(key))
        }
    }
}

tasks.processResources {
    from("LICENSE-vl53l1x.txt")
    from("LICENSE-vl53l0x.txt")
    from("LICENSE-vl6180x.txt")
    from("LICENSE.txt")
}

release {
    tagTemplate = "v\${version}"
}

publishing {
    publications {
        register<MavenPublication>("library") {
            from(components["java"])
        }
    }
    repositories {
        maven {
            val releasesRepoUrl = "https://maven.octyl.net/repository/armabot-release"
            val snapshotsRepoUrl = "https://maven.octyl.net/repository/armabot-snapshots"
            name = "octylNet"
            url = uri(if (version.toString().endsWith("SNAPSHOT")) snapshotsRepoUrl else releasesRepoUrl)
            credentials(PasswordCredentials::class)
        }
    }
}
