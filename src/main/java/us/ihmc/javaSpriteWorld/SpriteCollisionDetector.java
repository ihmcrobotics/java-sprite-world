package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygonIntersectionDetector;

public class SpriteCollisionDetector
{
   private final ConvexPolygonIntersectionDetector intersectionDetector = new ConvexPolygonIntersectionDetector();

   private final ArrayList<ConvexPolygon> collisionPolygonsOne = new ArrayList<>();
   private final ArrayList<ConvexPolygon> collisionPolygonsTwo = new ArrayList<>();

   public boolean areSpritesColliding(Sprite spriteOne, Sprite spriteTwo)
   {
      if (!spriteOne.isVisible()) return false;
      if (!spriteTwo.isVisible()) return false;

      collisionPolygonsOne.clear();
      spriteOne.getCollisionPolygons(collisionPolygonsOne);

      collisionPolygonsTwo.clear();
      spriteTwo.getCollisionPolygons(collisionPolygonsTwo);

      for (int m = 0; m < collisionPolygonsOne.size(); m++)
      {
         ConvexPolygon polygonOne = collisionPolygonsOne.get(m);

         for (int n = 0; n < collisionPolygonsTwo.size(); n++)
         {
            ConvexPolygon polygonTwo = collisionPolygonsTwo.get(n);

            //            System.out.println("Checking if " + polygonOne + " is intersecting with " + polygonTwo);
            if (intersectionDetector.arePolygonsIntersecting(polygonOne, polygonTwo))
               return true;
         }
      }

      return false;
   }
}
