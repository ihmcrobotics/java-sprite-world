package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Flag;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Food01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.Predator01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRobot;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

public class Robot02 implements RobotChallengeRobot
{
   private final Sprite sprite;
   private final double xMax, yMax;

   private double x, y;
   private double health;
   private double heading, velocity;
   private double acceleration, turnRate;
   private double maximumVelocity = 3.0;

   private Flag flagHolding;

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

      setHealth(100.0);

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

   public double getHealth()
   {
      return health;
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
      velocity += acceleration * dt;
      heading += turnRate * dt;

      if (velocity > maximumVelocity * health / 100.0)
      {
         velocity = maximumVelocity * health / 100.0;
      }
      if (velocity < 0.0)
         velocity = 0.0;

      double xDot = -velocity * Math.sin(heading);
      double yDot = velocity * Math.cos(heading);

      x += xDot * dt;
      y += yDot * dt;

      sprite.setX(x);
      sprite.setY(y);

      sprite.setRotationInRadians(heading);

      health = health - 1.0 * velocity * dt;
      if (health < 0.01)
         health = 0.01;
   }
   
   @Override
   public void teleportHome()
   {
      setX(0.5);
      setY(0.5);
      setVelocity(0.0);
      setHeading(0.0);
   }

   @Override
   public void eatFood(Food01 food)
   {
      health = health + 5.0;
      if (health > 100.0)
         health = 100.0;

      System.out.println("Yummy food! Health = " + health);
   }

   @Override
   public void getHitByPredator(Predator01 predator)
   {
      health = health - 5.0;
      if (health < 1.0)
         health = 1.0;

      System.out.println("Ouch! Health = " + health);
   }

   @Override
   public Point2D getPosition()
   {
      return new Point2D(x, y);
   }

   @Override
   public Vector2D getVelocityVector()
   {
      double xDot = -velocity * Math.sin(heading);
      double yDot = velocity * Math.cos(heading);

      return new Vector2D(xDot, yDot);
   }

   @Override
   public Vector2D getHeadingVector()
   {
      double x = -Math.sin(heading);
      double y = Math.cos(heading);

      return new Vector2D(x, y);
   }

   @Override
   public Flag dropFlag()
   {
      Flag flagToDrop = flagHolding;

      flagHolding = null;
      return flagToDrop;
   }

   @Override
   public void capturedFlag(Flag flag)
   {
      this.flagHolding = flag;
   }

   @Override
   public void hitWall()
   {
      health = health - 5.0;
      if (health < 1.0)
         health = 1.0;
      
      System.out.println("Hit the wall. Ouch! Health = " + health);
      
      heading = heading + Math.PI;
      double xDot = -velocity * Math.sin(heading);
      double yDot = velocity * Math.cos(heading);
      
      Vector2D unitVelocity = new Vector2D(xDot, yDot);
      unitVelocity.normalize();
      unitVelocity.scale(0.05 * xMax);
      x = x + unitVelocity.getX();
      y = y + unitVelocity.getY();
      
      velocity = 0.0;
      
      sprite.setX(x);
      sprite.setY(y);

      sprite.setRotationInRadians(heading);
   }

}
