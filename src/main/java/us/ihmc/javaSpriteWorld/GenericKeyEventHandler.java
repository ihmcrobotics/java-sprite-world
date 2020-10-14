package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

public class GenericKeyEventHandler
{
   private final SpriteWorldViewer viewer;
   private final SpriteWorld spriteWorld;
   
   public GenericKeyEventHandler(SpriteWorldViewer viewer, SpriteWorld spriteWorld)
   {
      this.viewer = viewer;
      this.spriteWorld = spriteWorld;
   }

   public void keyPressed(String character)
   {
      ArrayList<SpriteWorldKeyListener> spriteWorldKeyListeners = spriteWorld.getSpriteWorldKeyListeners();
      for (SpriteWorldKeyListener spriteWorldKeyListener : spriteWorldKeyListeners)
      {
         spriteWorldKeyListener.KeyPressed(viewer, spriteWorld, character);
      }      
   }

   public void keyReleased(String character)
   {
      ArrayList<SpriteWorldKeyListener> spriteWorldKeyListeners = spriteWorld.getSpriteWorldKeyListeners();
      for (SpriteWorldKeyListener spriteWorldKeyListener : spriteWorldKeyListeners)
      {
         spriteWorldKeyListener.KeyReleased(viewer, spriteWorld, character);
      } 
   }

   public void keyTyped(String character)
   {
      ArrayList<SpriteWorldKeyListener> spriteWorldKeyListeners = spriteWorld.getSpriteWorldKeyListeners();
      for (SpriteWorldKeyListener spriteWorldKeyListener : spriteWorldKeyListeners)
      {
         spriteWorldKeyListener.KeyTyped(viewer, spriteWorld, character);
      } 
   }
}
