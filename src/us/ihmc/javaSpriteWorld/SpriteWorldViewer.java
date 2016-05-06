package us.ihmc.javaSpriteWorld;

import javax.swing.JButton;

public interface SpriteWorldViewer
{
   public abstract void setPreferredSizeInPixels(int preferredWidth, int preferredHeight);
   public abstract void setSpriteWorld(SpriteWorld spriteWorld);
   public abstract void setResizable(boolean resizable);
   public abstract void createAndDisplayWindow();
   public abstract void addButton(JButton button);
   public abstract void update();
}