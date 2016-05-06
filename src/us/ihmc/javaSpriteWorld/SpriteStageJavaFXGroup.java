package us.ihmc.javaSpriteWorld;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;

public class SpriteStageJavaFXGroup extends Group
{
   private final SpriteStage spriteStage;
   private final  SpriteWorld spriteWorld;
   
   private StageBackdrop currentStageBackdrop;
   
   private final Translate translateForSpritePosition = new Translate();
   private final Scale spriteReflection = new Scale();
   private final Rotate spriteRotation = new Rotate();
   private final Scale scaleForSpriteSizeToImageSize = new Scale();
   private final Translate translateForImageCenter = new Translate();
   
   public SpriteStageJavaFXGroup(SpriteStage spriteStage, SpriteWorld spriteWorld)
   {
      super();
      this.spriteStage = spriteStage;
      this.spriteWorld = spriteWorld;
   }
   
   public SpriteStage getSprite()
   {
      return spriteStage;
   }
   
   public void update()
   {
      if (spriteStage == null) return;
      
      StageBackdrop backdrop = spriteStage.getBackdrop();
      
      if (currentStageBackdrop != backdrop)
      {
         ObservableList<Node> children = this.getChildren();
         children.clear();
         children.add(backdrop.getJavaFXNode());

         currentStageBackdrop = backdrop;
      }
      
      ObservableList<Transform> transforms = this.getTransforms();
      transforms.clear();
      
      double imageWidthPixels = backdrop.getImageWidthPixels();
      double imageHeightPixels = backdrop.getImageHeightPixels();

//      double reflectX = 1.0;
//      double reflectY = 1.0;
//      
//      if (sprite.getReflectX()) reflectX = -1.0;
//      if (sprite.getReflectY()) reflectY = -1.0;
      
//      translateForSpritePosition.setX(sprite.getX());
//      translateForSpritePosition.setY(sprite.getY());
//      transforms.add(translateForSpritePosition);
      
//      spriteReflection.setX(reflectX);
//      spriteReflection.setY(reflectY);
//      transforms.add(spriteReflection);
          
//      spriteRotation.setAngle(sprite.getRotationInDegrees());
//      transforms.add(spriteRotation);
      
      scaleForSpriteSizeToImageSize.setX(spriteWorld.getWidth() / imageWidthPixels);
      scaleForSpriteSizeToImageSize.setY(spriteWorld.getHeight() / imageHeightPixels);
      transforms.add(scaleForSpriteSizeToImageSize);

//      translateForImageCenter.setX(-backdrop.getImageCenterX());
//      translateForImageCenter.setY(-backdrop.getImageCenterY());
//      transforms.add(translateForImageCenter);
   }
}
