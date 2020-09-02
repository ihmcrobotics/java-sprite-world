package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Food01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRobot;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Robot02 implements RobotChallengeRobot
{
   private final Sprite sprite;
   private final double xMax, yMax;

   private double x, y;
   private double heading, velocity;
   private double acceleration, turnRate;

   public Robot02(double xMax, double yMax)
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

   public double getHeading()
   {
      return heading;
   }

   public void setHeading(double heading)
   {
      this.heading = heading;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public double getAcceleration()
   {
      return acceleration;
   }

   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
   }

   public double getTurnRate()
   {
      return turnRate;
   }

   public void setTurnRate(double turnRate)
   {
      this.turnRate = turnRate;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public void doDynamicsAndUpdateSprite(double dt)
   {
      velocity += acceleration * dt;
      heading += turnRate * dt;

      double xDot = -velocity * Math.sin(heading);
      double yDot = velocity * Math.cos(heading);

      x += xDot * dt;
      y += yDot * dt;

      if (isOutOfBounds())
      {
         teleportHome();
      }

      sprite.setX(x);
      sprite.setY(y);

      sprite.setRotationInRadians(heading);
   }

   private void teleportHome()
   {
      setX(0.5);
      setY(0.5);
      setVelocity(0.0);
      setHeading(0.0);
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
