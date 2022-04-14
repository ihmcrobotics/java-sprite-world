plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.3"
   vcsUrl = "https://stash.ihmc.us/projects/ICSL/repos/javaspriteworld"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-lang3:3.12.0")
   api("commons-io:commons-io:2.11.0")
}

testDependencies {
   api("us.ihmc:euclid:0.17.2")
   api("us.ihmc:euclid-geometry:0.17.2")
}
