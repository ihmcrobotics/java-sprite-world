plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "6.8"
   id("us.ihmc.ihmc-cd") version "1.14"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.2"
   vcsUrl = "https://stash.ihmc.us/projects/ICSL/repos/javaspriteworld"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-lang3:3.7")
   api("commons-io:commons-io:2.6")
}

testDependencies {
   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:euclid-geometry:0.15.1")
}
