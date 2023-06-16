plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.24"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.3"
   vcsUrl = "https://bitbucket.ihmc.us/projects/LIBS/repos/java-sprite-world"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-lang3:3.12.0")
   api("commons-io:commons-io:2.11.0")

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
   api(ihmc.javaFXModule("swing", javaFXVersion))
}

testDependencies {
   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:euclid-geometry:0.20.0")
}
