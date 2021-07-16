package us.ihmc.javaSpriteWorld;

public interface SpriteWorldKeyListener
{
   public abstract void KeyPressed(SpriteWorldViewer viewer, SpriteWorld spriteWorld, String character);
   public abstract void KeyReleased(SpriteWorldViewer viewer, SpriteWorld spriteWorld, String character);
   public abstract void KeyTyped(SpriteWorldViewer viewer, SpriteWorld spriteWorld, String character);

}
