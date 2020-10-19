package us.ihmc.javaSpriteWorld;

import javax.swing.JButton;

import javafx.scene.control.Button;

public interface SpriteWorldViewer
{
   public abstract void setPreferredSizeInPixels(int preferredWidth, int preferredHeight);
   public abstract void setSpriteWorld(SpriteWorld spriteWorld);
   public abstract SpriteWorld getSpriteWorld();
   public abstract void setResizable(boolean resizable);
   public abstract void createAndDisplayWindow();
   public abstract void addButton(JButton button);
   public abstract void addButton(Button button);
   public abstract void update();
   public abstract void setLocationOnScreen(int x, int y);
   public abstract void exit();
}