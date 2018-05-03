package us.ihmc.javaSpriteWorld;

public interface SpriteWorldMouseListener
{
   public abstract void mouseEnteredWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
   public abstract void mouseMovedInWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
   public abstract void mouseExitedWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
   
   public abstract void mouseClickedInWorld(SpriteWorld spriteWorld, double xWorld, double yWorld, int clickCount);
   public abstract void mousePressedInWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
   public abstract void mouseReleasedInWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
   public abstract void mouseDraggedInWorld(SpriteWorld spriteWorld, double xWorld, double yWorld);
}
