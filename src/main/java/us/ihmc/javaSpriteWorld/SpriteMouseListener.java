package us.ihmc.javaSpriteWorld;

public interface SpriteMouseListener 
{
	public abstract void spriteClicked(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld, int clickCount);
	public abstract void spritePressed(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld);
	public abstract void spriteReleased(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld);
	public abstract void spriteDragged(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld);
}
