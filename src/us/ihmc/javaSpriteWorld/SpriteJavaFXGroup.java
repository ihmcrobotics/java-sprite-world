package us.ihmc.javaSpriteWorld;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;

public class SpriteJavaFXGroup extends Group
{
   private final Sprite sprite;
   private SpriteCostume currentCostume;
   
   private final Translate translateForSpritePosition = new Translate();
   private final Rotate spriteRotation = new Rotate();
   private final Scale spriteReflection = new Scale();
   private final Scale scaleForSpriteSizeToImageSize = new Scale();
   private final Translate translateForImageCenter = new Translate();
   
   public SpriteJavaFXGroup(Sprite sprite)
   {
      super();
      this.sprite = sprite;
      
      ObservableList<Transform> transforms = this.getTransforms();
      transforms.clear();
      
      transforms.add(translateForSpritePosition);
      transforms.add(spriteRotation);
      transforms.add(spriteReflection);
      transforms.add(scaleForSpriteSizeToImageSize);
      transforms.add(translateForImageCenter);
   }
   
   public Sprite getSprite()
   {
      return sprite;
   }
   
   public void update()
   {
      SpriteCostume costume = sprite.getCostume();
      if (sprite.isHidden()) costume = null;
      
      if (currentCostume != costume)
      {
         ObservableList<Node> children = this.getChildren();
         Node javaFXNode = null;
         
         if (costume != null) javaFXNode = costume.getJavaFXNode();

         boolean foundIt = false;
         
         for (Node child : children)
         {
            if (child == javaFXNode) 
            {
               foundIt = true;
               child.setVisible(true);
            }
            else child.setVisible(false);
         }
         
         if ((!foundIt) && (javaFXNode != null))
         {
            children.add(javaFXNode);
         }

         currentCostume = costume;
      }
      
      if (currentCostume == null) return;
      
      double imageWidthPixels = costume.getImageWidthPixels();
      double imageHeightPixels = costume.getImageHeightPixels();

      double reflectX = 1.0;
      double reflectY = 1.0;
      
      if (sprite.getReflectX()) reflectX = -1.0;
      if (sprite.getReflectY()) reflectY = -1.0;
      
      translateForSpritePosition.setX(sprite.getX());
      translateForSpritePosition.setY(sprite.getY());
      
      spriteRotation.setAngle(sprite.getRotationInDegrees());
      
      spriteReflection.setX(reflectX);
      spriteReflection.setY(reflectY);
      
      scaleForSpriteSizeToImageSize.setX(sprite.getWidth() / imageWidthPixels);
      scaleForSpriteSizeToImageSize.setY(sprite.getHeight() / imageHeightPixels);

      translateForImageCenter.setX(-costume.getImageCenterX());
      translateForImageCenter.setY(-costume.getImageCenterY());
   }
}
