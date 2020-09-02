package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Robot01
{
   private final Sprite sprite;
   private final double xMax, yMax;

   private double x, y;
   private double xDot, yDot;

   public Robot01(double xMax, double yMax)
   {
      this.xMax = xMax;
      this.yMax = yMax;
      
      sprite = SampleSprites.createRocketOne();
      sprite.setReflectY(true);
      sprite.setWidth(0.5);
      sprite.setHeight(1.0);

      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.25, -0.5), new Vector(0.5, 1.0));
      sprite.addCollisionPolygon(collisionPolygon);

      teleportHome();
   }

   public double getX()
   {
      return x;
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public double getY()
   {
      return y;
   }

   public void setY(double y)
   {
      this.y = y;
   }


   public double getXDot()
   {
      return xDot;
   }
   
   public double getYDot()
   {
      return yDot;
   }

   public void setXDot(double xDot)
   {
      this.xDot = xDot;
   }
   
   public void setYDot(double yDot)
   {
      this.yDot = yDot;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public void doDynamicsAndUpdateSprite(double dt)
   {
      x += xDot * dt;
      y += yDot * dt;

      if (isOutOfBounds())
      {
         teleportHome();
      }

      sprite.setX(x);
      sprite.setY(y);
   }

   private void teleportHome()
   {
      setX(0.5);
      setY(0.5);
   }

   private boolean isOutOfBounds()
   {
      if (x > xMax)
         return true;
      if (y > yMax)
         return true;

      if (x < 0.0)
         return true;
      if (y < 0.0)
         return true;

      return false;
   }

   public void eatFood(Food01 food)
   {
      System.out.println("Yummy food!");
      
   }

}
