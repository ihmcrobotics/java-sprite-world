package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Predator01
{
   private final double xMax, yMax;
   private final Sprite sprite;
   private final Random random;

   private double x, y;
   private double heading, speed;
   private double acceleration, turnRate;

   private final double maximumSpeed;
   
   public Predator01(Random random, double xMax, double yMax, double maximumSpeed)
   {
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;
      this.maximumSpeed = maximumSpeed;

      sprite = SampleSprites.createSoccerBall("Predator");
      sprite.setHeightPreserveScale(0.3, 0);

      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.15, -0.15), new Vector(0.3, 0.3));
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

   public double getSpeed()
   {
      return speed;
   }

   public void setSpeed(double speed)
   {
      this.speed = speed;
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

   public void doDynamicsAndUpdateSprite(Point2D robotPosition, Vector2D robotVelocity, double dt)
   {
      Point2D myPosition = new Point2D(x, y);
      Vector2D meToRobot = new Vector2D(robotPosition);
      meToRobot.sub(myPosition);

      Vector2D myHeadingVector = new Vector2D(1.0, 0.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setToYawOrientation(heading);
      transform.transform(myHeadingVector);

      double cross = meToRobot.cross(myHeadingVector);
      double dot = meToRobot.dot(myHeadingVector);
      
      if (dot < 0.0)
      {
         acceleration = -1.0;

      }
      else
      {
         acceleration = 1.0;
      }
      
      turnRate = -0.2 * maximumSpeed * cross;

      speed += acceleration * dt;
      heading += turnRate * dt;

      if (speed > maximumSpeed)
         speed = maximumSpeed;

      if (speed < 0.0)
         speed = 0.0;

      double xDot = speed * Math.cos(heading);
      double yDot = speed * Math.sin(heading);

      x += xDot * dt;
      y += yDot * dt;

//      if (isOutOfBounds())
//      {
//         teleportToRandomLocation(random);
//      }

      sprite.setX(x);
      sprite.setY(y);

      sprite.setRotationInRadians(heading);
   }

   public void teleportToRandomLocation(Random random)
   {
      setX(random.nextDouble() * xMax);
      setY(random.nextDouble() * yMax);

      setSpeed(random.nextDouble() * 1.0);
      setHeading(random.nextDouble() * 2.0 * Math.PI);
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
