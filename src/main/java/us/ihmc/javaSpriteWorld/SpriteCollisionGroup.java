package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

public class SpriteCollisionGroup
{
   private final ArrayList<Sprite> sprites = new ArrayList<>();
   private final ArrayList<SpriteCollisionListener> spriteCollisionListeners = new ArrayList<>();

   private final SpriteCollisionDetector spriteCollisionDetector = new SpriteCollisionDetector();
   
   public void addSprite(Sprite sprite)
   {
      this.sprites.add(sprite);
   }

   public void addSpriteCollisionListener(SpriteCollisionListener listener)
   {
      this.spriteCollisionListeners.add(listener);
   }

   public boolean doCheckCollisions()
   {
      boolean someSpritesAreColliding = false;
      
      for (int i = 0; i < sprites.size(); i++)
      {
         Sprite spriteOne = sprites.get(i);

         for (int j = i + 1; j < sprites.size(); j++)
         {
            Sprite spriteTwo = sprites.get(j);
//            System.out.println("Checking if " + spriteOne.getName() + " is colliding with " + spriteTwo.getName());
            boolean spritesAreColliding = spriteCollisionDetector.areSpritesColliding(spriteOne, spriteTwo);

            if (spritesAreColliding)
            {
               someSpritesAreColliding = true;
//               System.out.println(spriteOne.getName() + " is colliding with " + spriteTwo.getName());

               notifyListenersThatSpritesAreColliding(spriteOne, spriteTwo);
            }
         }
      }
      
      return someSpritesAreColliding;
   }

   private void notifyListenersThatSpritesAreColliding(Sprite spriteOne, Sprite spriteTwo)
   {  
      for (int i = 0; i < spriteCollisionListeners.size(); i++)
      {
         spriteCollisionListeners.get(i).spritesAreColliding(spriteOne, spriteTwo);
      }
   }

}
