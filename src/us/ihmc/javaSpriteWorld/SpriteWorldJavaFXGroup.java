package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;

public class SpriteWorldJavaFXGroup extends Group
{
   private SpriteWorld spriteWorld;
   private final HashMap<Sprite, SpriteJavaFXGroup> spriteJavaFXGroups = new HashMap<>();
   private SpriteStageJavaFXGroup spriteStageJavaFXGroup = null;
   
   private final Scale scaleForWorldSize = new Scale();
   private final Translate translateForTopLeftCorner = new Translate();
      
   public SpriteWorldJavaFXGroup(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;
      
      this.getTransforms().clear();
      this.getTransforms().add(scaleForWorldSize);
      this.getTransforms().add(translateForTopLeftCorner);
      
      MouseClickedHandler mouseClickedHandler = new MouseClickedHandler();
      MouseDraggedHandler mouseDraggedHandler = new MouseDraggedHandler();
      MousePressedHandler mousePressedHandler = new MousePressedHandler();
      MouseReleasedHandler mouseReleasedHandler = new MouseReleasedHandler();
//      MouseReleasedHandler mouseMovedHandler = new MouseMovedHandler();
//      MouseReleasedHandler mouseExitedHandler = new MouseExitedHandler();
//      MouseReleasedHandler mouseEnteredHandler = new MouseEnteredHandler();

      
      this.setOnMouseClicked(mouseClickedHandler);
//      this.setOnMouseMoved(mouseMovedHandler);
      this.setOnMouseDragged(mouseDraggedHandler);
//    this.setOnMouseEntered(mouseEnteredHandler);
//    this.setOnMouseExited(mouseExitedHandler);
      this.setOnMousePressed(mousePressedHandler);
      this.setOnMouseReleased(mouseReleasedHandler);
   }
   
   public void update()
   {      
      if (spriteStageJavaFXGroup == null)
      {
         SpriteStage spriteStage = spriteWorld.getSpriteStage();
         spriteStageJavaFXGroup = new SpriteStageJavaFXGroup(spriteStage, spriteWorld);
         this.getChildren().add(spriteStageJavaFXGroup);
      }
      
      spriteStageJavaFXGroup.update();
      
      List<Sprite> sprites = spriteWorld.getSprites();
      
      for (Sprite sprite : sprites)
      {
//         if (sprite.isHidden()) continue;
         
         SpriteJavaFXGroup spriteJavaFXGroup = spriteJavaFXGroups.get(sprite);
         
         if (spriteJavaFXGroup == null)
         {
            spriteJavaFXGroup = new SpriteJavaFXGroup(sprite);
            spriteJavaFXGroups.put(sprite, spriteJavaFXGroup);
            this.getChildren().add(spriteJavaFXGroup);
         }
         
         spriteJavaFXGroup.update();
      }
      
      double leftBorderX = spriteWorld.getLeftBorderX();
      double rightBorderX = spriteWorld.getRightBorderX();

      double topBorderY = spriteWorld.getTopBorderY();
      double bottomBorderY = spriteWorld.getBottomBorderY();

      scaleForWorldSize.setX(1.0/(rightBorderX - leftBorderX));
      scaleForWorldSize.setY(1.0/(bottomBorderY - topBorderY));

      translateForTopLeftCorner.setX(-leftBorderX);
      translateForTopLeftCorner.setY(-topBorderY);
   }
   
   public void setSpriteWorld(SpriteWorld spriteWorld) 
   {
      this.spriteWorld = spriteWorld;  
   }

   private Sprite findTopMostSpriteWithMouseListenersAt(double worldX, double worldY)
   {
      List<Sprite> sprites = spriteWorld.getSprites();
      
      for (int i=sprites.size() - 1; i>=0; i--)
      {
         Sprite sprite = sprites.get(i);
         
         if (sprite.getSpriteMouseListeners().size() > 0)
         {
            if (sprite.isClickPointInside(worldX, worldY)) return sprite;
         }
      }
      
      return null;
   }
   
   private Sprite spriteCurrentlyBeingDragged = null;
   
   private class MouseDraggedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseDragged(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }
   
   private class MouseClickedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         MouseButton button = mouseEvent.getButton();
         mouseClicked(mouseEvent.getX(), mouseEvent.getY(), mouseEvent.getClickCount(), mouseEvent);
         mouseEvent.consume();
      }
   }
   
   private class MousePressedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mousePressed(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }
   
   private class MouseReleasedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseReleased(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }
   
   public void mouseDragged(double xWorld, double yWorld) 
   {     
//      System.out.println("Mouse Dragged! " + xWorld + spriteCurrentlyBeingDragged);

      Sprite sprite = spriteCurrentlyBeingDragged;
      
      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
         
         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteDragged(sprite, xWorld, yWorld);
         }
      }
   }

   public void mouseClicked(double xWorld, double yWorld, int clickCount, MouseEvent mouseEvent) 
   {
      Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
//      System.out.println("Mouse Clicked on sprite " + sprite + " at " + xWorld + ", " + yWorld);
      
      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteClicked(sprite, xWorld, yWorld, mouseEvent);
         }
      }
   }

   public void mousePressed(double xWorld, double yWorld) 
   {
      Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
      
      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
         
         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spritePressed(sprite, xWorld, yWorld);
         }
      }
      
      spriteCurrentlyBeingDragged = sprite;
   }

   public void mouseReleased(double xWorld, double yWorld) 
   {
      Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
      
      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
         
         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteReleased(sprite, xWorld, yWorld);
         }
      }
      
      spriteCurrentlyBeingDragged = null;
   }

}