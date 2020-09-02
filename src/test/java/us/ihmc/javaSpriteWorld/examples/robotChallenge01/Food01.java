package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.Random;

import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Food01
{
   private final double xMax, yMax;
   private final Sprite sprite;
   private final Random random;
   
   private double x, y;
   private double heading, velocity;
   private double acceleration, turnRate;

   public Food01(Random random, double xMax, double yMax)
   {
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;

      sprite = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      sprite.setHeightPreserveScale(0.5, 0);

      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.25, -0.25), new Vector(0.5, 0.5));
      sprite.addCollisionPolygon(collisionPolygon);

      teleportToRandomLocation(random);
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
      acceleration = -1.0 + 2.0 * random.nextDouble();
      turnRate = -1.0 + 2.0 * random.nextDouble();
      
      velocity += acceleration * dt;
      heading += turnRate * dt;

      double xDot = -velocity * Math.sin(heading);
      double yDot = velocity * Math.cos(heading);

      x += xDot * dt;
      y += yDot * dt;

      if (isOutOfBounds())
      {
         teleportToRandomLocation(random);
      }

      sprite.setX(x);
      sprite.setY(y);

      sprite.setRotationInRadians(heading);
   }

   public void teleportToRandomLocation(Random random)
   {
      setX(random.nextDouble() * xMax);
      setY(random.nextDouble() * yMax);

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

}
