package us.ihmc.javaSpriteWorld.javaFX;

import us.ihmc.javaSpriteWorld.SpriteStage;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class SpriteStageJavaFXGroup extends SpriteJavaFXGroup
{
   private final SpriteStage spriteStage;
   private final  SpriteWorld spriteWorld;   
 
   public SpriteStageJavaFXGroup(SpriteStage spriteStage, SpriteWorld spriteWorld)
   {
      super(spriteStage);
      this.spriteStage = spriteStage;
      this.spriteWorld = spriteWorld;
   }

}
