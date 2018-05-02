package us.ihmc.javaSpriteWorld;

import javafx.scene.input.MouseEvent;

public interface SpriteMouseListener 
{
	public abstract void spriteClicked(Sprite sprite, double xWorld, double yWorld, MouseEvent mouseEvent);
	public abstract void spritePressed(Sprite sprite, double xWorld, double yWorld);
	public abstract void spriteReleased(Sprite sprite, double xWorld, double yWorld);
	public abstract void spriteDragged(Sprite sprite, double xWorld, double yWorld);
}
