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

   private Flag flagHolding;

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

      reset();
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

   @Override
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

   public Flag getFlagHolding()
   {
      return flagHolding;
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

      sprite.setX(x);
      sprite.setY(y);

      health = health - 1.0 * velocity.length() * dt;
      if (health < 1.0)
         health = 1.0;
   }

   @Override
   public void teleportHome()
   {
      setX(1.5);
      setY(1.5);
   }

   @Override
   public void eatFood(Food01 food)
   {
      health = health + 1.0;
      if (health > 100.0)
         health = 100.0;

//      System.out.println("Yummy food! Health = " + health);
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
   
   @Override
   public Vector2D getHeadingVector()
   {
      Vector2D velocityVector = getVelocityVector();
      if (velocityVector.length() < 0.0001)
      {
         return new Vector2D(1.0, 0.0);
      }
      
      velocityVector.normalize();
      return velocityVector;
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
//      System.out.println("Hit the wall. Ouch! Health = " + health);
      health = health - 5.0;
      if (health < 1.0)
         health = 1.0;
      
      if (Math.abs(xDot) < 0.01 & (Math.abs(yDot) < 0.01)) 
         return;
      
      Vector2D unitVelocity = new Vector2D(xDot, yDot);
      unitVelocity.normalize();
      unitVelocity.scale(-0.05 * xMax);
      x = x + unitVelocity.getX();
      y = y + unitVelocity.getY();
      
      xDot = 0.0;
      yDot = 0.0; 
   }

   @Override
   public void reset()
   {
      setHealth(100.0);
      teleportHome();
   }

}
