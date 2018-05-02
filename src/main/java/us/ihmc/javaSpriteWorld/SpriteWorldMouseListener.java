package us.ihmc.javaSpriteWorld;

import javafx.scene.input.MouseEvent;

public interface SpriteWorldMouseListener
{
   public abstract void worldClicked(double xWorld, double yWorld, MouseEvent mouseEvent);
   public abstract void worldPressed(double xWorld, double yWorld);
   public abstract void worldReleased(double xWorld, double yWorld);
   public abstract void worldDragged(double xWorld, double yWorld);
}
