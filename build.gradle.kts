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

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))

   val gdxVersion = "1.10.0"
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:$gdxVersion")
   api("com.badlogicgames.gdx:gdx-platform:$gdxVersion:natives-desktop")

   val imguiVersion = "1.84.1.4"
   api("io.github.spair:imgui-java-binding:$imguiVersion")
   api("io.github.spair:imgui-java-lwjgl3:$imguiVersion")
   api("io.github.spair:imgui-java-natives-linux-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-macos-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-windows-ft:$imguiVersion")
}

testDependencies {
   api(ihmc.sourceSetProject("libgdx"))

   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:euclid-geometry:0.17.0")
}
