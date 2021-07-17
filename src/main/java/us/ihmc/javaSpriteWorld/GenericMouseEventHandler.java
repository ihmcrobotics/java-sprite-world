package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;
import java.util.List;

public class GenericMouseEventHandler
{
   private static final boolean DEBUG = false;

   private final SpriteWorldViewer viewer;
   private final SpriteWorld spriteWorld;

   private Sprite spritePressed = null;
   private Sprite spriteCurrentlyBeingDragged = null;

   public GenericMouseEventHandler(SpriteWorldViewer viewer, SpriteWorld spriteWorld)
   {
      this.viewer = viewer;
      this.spriteWorld = spriteWorld;
   }

   private Sprite findTopMostSpriteWithMouseListenersAt(double worldX, double worldY)
   {
      List<Sprite> sprites = spriteWorld.getSprites();

      for (int i = sprites.size() - 1; i >= 0; i--)
      {
         Sprite sprite = sprites.get(i);

         if (sprite.getSpriteMouseListeners().size() > 0)
         {
            if (sprite.isClickPointInside(worldX, worldY))
               return sprite;
         }
      }

      return null;
   }

   public void mouseEntered(double xWorld, double yWorld)
   {
      if (DEBUG)
         System.out.println("Mouse entered : " + xWorld + ", " + yWorld);

      ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
      for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
      {
         spriteWorldMouseListener.mouseEnteredWorld(viewer, spriteWorld, xWorld, yWorld);
      }
   }

   public void mouseMoved(double xWorld, double yWorld)
   {
      if (DEBUG)
         System.out.println("Mouse moved : " + xWorld + ", " + yWorld);

      ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
      for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
      {
         spriteWorldMouseListener.mouseMovedInWorld(viewer, spriteWorld, xWorld, yWorld);
      }
   }

   public void mouseExited(double xWorld, double yWorld)
   {
      if (DEBUG)
         System.out.println("Mouse exited : " + xWorld + ", " + yWorld);

      ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
      for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
      {
         spriteWorldMouseListener.mouseExitedWorld(viewer, spriteWorld, xWorld, yWorld);
      }
   }

   public void mouseClicked(double xWorld, double yWorld, int clickCount)
   {
      Sprite sprite = spritePressed; //findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);

      if (DEBUG)
         System.out.println("Mouse Clicked on sprite " + sprite + " at " + xWorld + ", " + yWorld);

      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteClicked(viewer, sprite, xWorld, yWorld, clickCount);
         }
      }

      else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.mouseClickedInWorld(viewer, spriteWorld, xWorld, yWorld, clickCount);
         }
      }
   }

   public void mousePressed(double xWorld, double yWorld)
   {
      Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
      spritePressed = sprite;

      if (DEBUG)
         System.out.println("Mouse Pressed on sprite " + sprite + " at " + xWorld + ", " + yWorld);

      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();

         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spritePressed(viewer, sprite, xWorld, yWorld);
         }
      }

      else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.mousePressedInWorld(viewer, spriteWorld, xWorld, yWorld);
         }
      }
   }

   public void mouseReleased(double xWorld, double yWorld)
   {
      if (DEBUG)
         System.out.println("Mouse Released. spriteCurrentlyBeingDragged = " + spriteCurrentlyBeingDragged + " at " + xWorld + ", " + yWorld);

      if (spriteCurrentlyBeingDragged != null)
      {
         Sprite sprite = spriteCurrentlyBeingDragged;
         spriteCurrentlyBeingDragged = null;

         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();

         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteReleased(viewer, sprite, xWorld, yWorld);
         }
      }
      else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.mouseReleasedInWorld(viewer, spriteWorld, xWorld, yWorld);
         }
      }
   }

   public void mouseDragged(double xWorld, double yWorld)
   {
      if (DEBUG)
         System.out.println("Mouse Dragged! " + spriteCurrentlyBeingDragged);

      spriteCurrentlyBeingDragged = spritePressed;
      Sprite sprite = spriteCurrentlyBeingDragged;

      if (sprite != null)
      {
         ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();

         for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
         {
            spriteMouseListener.spriteDragged(viewer, sprite, xWorld, yWorld);
         }
      }

      else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.mouseDraggedInWorld(viewer, spriteWorld, xWorld, yWorld);
         }
      }
   }

}
