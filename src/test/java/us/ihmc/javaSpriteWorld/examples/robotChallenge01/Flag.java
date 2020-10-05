package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Flag
{
   private final int id;
   private final Sprite sprite;
   private double x, y;

   public Flag(int id, double x, double y)
   {
      this.id = id;
      this.x = x;
      this.y = y;

      sprite = SampleSprites.createSixSidedRedPipsOnWhiteDie();
      sprite.setCostume((id - 1) % 6);
      sprite.setHeightPreserveScale(0.3, 0);

      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.15, -0.15), new Vector(0.3, 0.3));
      sprite.addCollisionPolygon(collisionPolygon);

      this.doDynamicsAndUpdateSprite(0.0);
   }

   public void setLocation(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public void setLocation(Point2D position)
   {
      this.x = position.getX();
      this.y = position.getY();
      doDynamicsAndUpdateSprite(0.0);
   }

   public double getX()
   {
      return x;
   }

   public double getY()
   {
      return y;
   }

   public int getId()
   {
      return id;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public void doDynamicsAndUpdateSprite(double dt)
   {
      sprite.setX(x);
      sprite.setY(y);
   }

}
