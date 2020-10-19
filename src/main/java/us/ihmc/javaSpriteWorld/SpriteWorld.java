package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public class SpriteWorld
{
   private SpriteStage spriteStage;
   private final CopyOnWriteArrayList<Sprite> sprites = new CopyOnWriteArrayList<Sprite>();

   private double leftBorderX = -1.0, rightBorderX = 1.0;
   private double topBorderY = -1.0, bottomBorderY = 1.0;

   private boolean worldScrollsX = false;
   private boolean worldScrollsY = false;

   private ArrayList<SpriteCollisionGroup> spriteCollisionGroups;
   private ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = new ArrayList<>();
   private ArrayList<SpriteWorldKeyListener> spriteWorldKeyListeners = new ArrayList<>();

   public SpriteWorld()
   {

   }

   public void attacheSpriteWorldMouseListener(SpriteWorldMouseListener spriteWorldMouseListener)
   {
      this.spriteWorldMouseListeners.add(spriteWorldMouseListener);
   }

   public ArrayList<SpriteWorldMouseListener> getSpriteWorldMouseListeners()
   {
      return spriteWorldMouseListeners;
   }

   public void attacheSpriteWorldKeyListener(SpriteWorldKeyListener spriteWorldKeyListener)
   {
      this.spriteWorldKeyListeners.add(spriteWorldKeyListener);
   }

   public ArrayList<SpriteWorldKeyListener> getSpriteWorldKeyListeners()
   {
      return spriteWorldKeyListeners;
   }

   public void addSpriteCollisionGroup(SpriteCollisionGroup spriteCollisionGroup)
   {
      if (spriteCollisionGroups == null)
      {
         spriteCollisionGroups = new ArrayList<>();
      }

      spriteCollisionGroups.add(spriteCollisionGroup);
   }

   public void checkSpriteCollisions()
   {
      if (spriteCollisionGroups == null)
         return;

      for (int i = 0; i < spriteCollisionGroups.size(); i++)
      {
         spriteCollisionGroups.get(i).doCheckCollisions();
      }
   }

   public SpriteWorldViewer createAndDisplaySpriteWorldViewerUsingSwing(String name, SpriteWorld spriteWorld, int preferredWidth, int preferredHeight)
   {
      SpriteWorldViewer viewer = new SpriteWorldViewerUsingSwing(name);
      viewer.setPreferredSizeInPixels(preferredWidth, preferredHeight);
      viewer.setResizable(false);
      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();

      return viewer;
   }

   public SpriteWorldViewerUsingJavaFX createAndDisplaySpriteWorldViewerUsingJavaFX(String name, SpriteWorld spriteWorld, int preferredWidth,
                                                                                    int preferredHeight)
   {
      SpriteWorldViewerUsingJavaFX viewer = new SpriteWorldViewerUsingJavaFX(name);
      viewer.setPreferredSizeInPixels(preferredWidth, preferredHeight);
      viewer.setResizable(false);
      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();

      return viewer;
   }

   public boolean getWorldScrollsX()
   {
      return worldScrollsX;
   }

   public void setWorldScrollsX(boolean worldScrollsX)
   {
      this.worldScrollsX = worldScrollsX;
   }

   public boolean getWorldScrollsY()
   {
      return worldScrollsY;
   }

   public void setWorldScrollsY(boolean worldScrollsY)
   {
      this.worldScrollsY = worldScrollsY;
   }

   public void addSprite(Sprite sprite)
   {
      if (!sprites.contains(sprite))
      {
         sprites.add(sprite);
      }
   }
   
   public boolean removeSprite(Sprite sprite)
   {
      return sprites.remove(sprite);
   }

   public void setStage(SpriteStage spriteStage, boolean centerStageInWorld)
   {
      this.spriteStage = spriteStage;
      
      if (centerStageInWorld)
      {
         spriteStage.setCenteredInSpriteWorld(this);
      }
   }

   public void setLeftBorderX(double leftBorderX)
   {
      this.leftBorderX = leftBorderX;
   }

   public void setRightBorderX(double rightBorderX)
   {
      this.rightBorderX = rightBorderX;
   }

   public void setTopBorderY(double topBorderY)
   {
      this.topBorderY = topBorderY;
   }

   public void setBottomBorderY(double bottomBorderY)
   {
      this.bottomBorderY = bottomBorderY;
   }

   public SpriteStage getSpriteStage()
   {
      return spriteStage;
   }

   public List<Sprite> getSprites()
   {
      return sprites;
   }

   public double getLeftBorderX()
   {
      return leftBorderX;
   }

   public double getTopBorderY()
   {
      return topBorderY;
   }

   public double getRightBorderX()
   {
      return rightBorderX;
   }

   public double getBottomBorderY()
   {
      return bottomBorderY;
   }

   public double getHeight()
   {
      return Math.abs(bottomBorderY - topBorderY);
   }

   public double getWidth()
   {
      return Math.abs(rightBorderX - leftBorderX);
   }

   public void start()
   {

   }


}
