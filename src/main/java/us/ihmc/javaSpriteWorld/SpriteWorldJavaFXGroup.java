package us.ihmc.javaSpriteWorld;

import java.util.HashMap;
import java.util.List;

import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.KeyEvent;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;

public class SpriteWorldJavaFXGroup extends Group
{
   private SpriteWorld spriteWorld;
   private GenericMouseEventHandler mouseEventHandler;
   private GenericKeyEventHandler keyEventHandler;

   private final HashMap<Sprite, SpriteJavaFXGroup> spriteJavaFXGroups = new HashMap<>();
   private SpriteStageJavaFXGroup spriteStageJavaFXGroup = null;

   private final Scale scaleForWorldSize = new Scale();
   private final Translate translateForTopLeftCorner = new Translate();

   public SpriteWorldJavaFXGroup(SpriteWorldViewer viewer, SpriteWorld spriteWorld)
   {
      setSpriteWorld(viewer, spriteWorld);

      this.getTransforms().clear();
      this.getTransforms().add(scaleForWorldSize);
      this.getTransforms().add(translateForTopLeftCorner);

      MouseEnteredHandler mouseEnteredHandler = new MouseEnteredHandler();
      MouseMovedHandler mouseMovedHandler = new MouseMovedHandler();
      MouseExitedHandler mouseExitedHandler = new MouseExitedHandler();
      MousePressedHandler mousePressedHandler = new MousePressedHandler();
      MouseDraggedHandler mouseDraggedHandler = new MouseDraggedHandler();
      MouseReleasedHandler mouseReleasedHandler = new MouseReleasedHandler();
      MouseClickedHandler mouseClickedHandler = new MouseClickedHandler();

      this.setOnMouseEntered(mouseEnteredHandler);
      this.setOnMouseMoved(mouseMovedHandler);
      this.setOnMouseExited(mouseExitedHandler);
      this.setOnMousePressed(mousePressedHandler);
      this.setOnMouseDragged(mouseDraggedHandler);
      this.setOnMouseReleased(mouseReleasedHandler);
      this.setOnMouseClicked(mouseClickedHandler);
      
      KeyPressedHandler keyPressedHandler = new KeyPressedHandler();
      KeyReleasedHandler keyReleasedHandler = new KeyReleasedHandler();
      KeyTypedHandler keyTypedHandler = new KeyTypedHandler();

      this.setOnKeyPressed(keyPressedHandler);
      this.setOnKeyReleased(keyReleasedHandler);
      this.setOnKeyTyped(keyTypedHandler);
   }

   public void setSpriteWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;
      this.mouseEventHandler = new GenericMouseEventHandler(viewer, spriteWorld);
      this.keyEventHandler = new GenericKeyEventHandler(viewer, spriteWorld);
   }

   private void printAndPause(String string, long pause)
   {
      System.out.println(string);
      try
      {
         Thread.sleep(pause);
      }
      catch (InterruptedException e)
      {
      }
   }
   
   public void update()
   {
      
//      printAndPause("In SpriteWorldJavaFXGroup.update()", 1000L);
//      boolean fxApplicationThread = Platform.isFxApplicationThread();

//      System.out.println("fxApplicationThread = " + fxApplicationThread);
      
      if (spriteStageJavaFXGroup == null)
      {
         SpriteStage spriteStage = spriteWorld.getSpriteStage();
         if (spriteStage != null)
         {
            spriteStageJavaFXGroup = new SpriteStageJavaFXGroup(spriteStage, spriteWorld);
            this.getChildren().add(spriteStageJavaFXGroup);
            spriteStageJavaFXGroup.update();
         }
      }

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

      scaleForWorldSize.setX(1.0 / (rightBorderX - leftBorderX));
      scaleForWorldSize.setY(1.0 / (bottomBorderY - topBorderY));

      translateForTopLeftCorner.setX(-leftBorderX);
      translateForTopLeftCorner.setY(-topBorderY);
      
//      printAndPause("Leaving SpriteWorldJavaFXGroup.update()", 1000L);

   }

   private class KeyPressedHandler implements EventHandler<KeyEvent>
   {
      @Override
      public void handle(KeyEvent keyEvent)
      {
         keyEventHandler.keyPressed(keyEvent.getCharacter());
         keyEvent.consume();
      }
   }
   
   private class KeyReleasedHandler implements EventHandler<KeyEvent>
   {
      @Override
      public void handle(KeyEvent keyEvent)
      {
         keyEventHandler.keyReleased(keyEvent.getCharacter());
         keyEvent.consume();
      }
   }

   private class KeyTypedHandler implements EventHandler<KeyEvent>
   {
      @Override
      public void handle(KeyEvent keyEvent)
      {
         keyEventHandler.keyTyped(keyEvent.getCharacter());
         keyEvent.consume();
      }
   }

   private class MouseEnteredHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseEntered(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MouseMovedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseMoved(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MouseExitedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseExited(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MousePressedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mousePressed(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MouseDraggedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseDragged(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MouseReleasedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseReleased(mouseEvent.getX(), mouseEvent.getY());
         mouseEvent.consume();
      }
   }

   private class MouseClickedHandler implements EventHandler<MouseEvent>
   {
      @Override
      public void handle(MouseEvent mouseEvent)
      {
         mouseEventHandler.mouseClicked(mouseEvent.getX(), mouseEvent.getY(), mouseEvent.getClickCount());
         mouseEvent.consume();
      }
   }

}