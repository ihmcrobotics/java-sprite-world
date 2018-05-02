package us.ihmc.javaSpriteWorld;

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
