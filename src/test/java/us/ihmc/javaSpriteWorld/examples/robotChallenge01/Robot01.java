package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Robot01 implements RobotChallengeRobot
{
   private final Sprite sprite;
   private final double xMax, yMax;

   private double x, y;
   private double health;
   private double xDot, yDot;
   private double maximumVelocity = 3.0;

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

      setHealth(100.0);

      teleportHome();
   }

   public double getX()
   {
      return x;
   }

   public double getY()
   {
      return y;
   }

   public double getXDot()
   {
      return xDot;
   }

   public double getYDot()
   {
      return yDot;
   }

   public double getHealth()
   {
      return health;
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public void setY(double y)
   {
      this.y = y;
   }

   public void setXDot(double xDot)
   {
      this.xDot = xDot;
   }

   public void setYDot(double yDot)
   {
      this.yDot = yDot;
   }

   public void setHealth(double health)
   {
      this.health = health;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public void doDynamicsAndUpdateSprite(double dt)
   {
      Vector2D velocity = new Vector2D(xDot, yDot);
      if (velocity.length() > maximumVelocity * health / 100.0)
      {
         velocity.normalize();
         velocity.scale(maximumVelocity * health / 100.0);
         xDot = velocity.getX();
         yDot = velocity.getY();
      }

      x += xDot * dt;
      y += yDot * dt;

      if (isOutOfBounds())
      {
         teleportHome();
      }

      sprite.setX(x);
      sprite.setY(y);

      health = health - 1.0 * velocity.length() * dt;
      if (health < 1.0)
         health = 1.0;
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

   @Override
   public void eatFood(Food01 food)
   {
      health = health + 1.0;
      if (health > 100.0)
         health = 100.0;

      System.out.println("Yummy food! Health = " + health);
   }

   @Override
   public void getHitByPredator(Predator01 predator)
   {
      health = health - 5.0;
      if (health < 1.0) health = 1.0;
   }

   @Override
   public Point2D getPosition()
   {
      return new Point2D(x, y);
   }

   @Override
   public Vector2D getVelocityVector()
   {
      return new Vector2D(xDot, yDot);
   }

}
