package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

public class SimpleRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
   private double dt = 0.001;
   
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double velocity = 0.0;
   private double heading = 0.0;
   private double x = 0.0, y = 0.0;
   private double xDotInWorld = 0.0;
   private double yDotInWorld = 0.0;

   public SimpleRobot05Behavior()
   {
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = heading;
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood)
   {
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      xDotInWorld = Math.cos(heading) * velocity;
      yDotInWorld = Math.sin(heading) * velocity;
      
      x = x + xDotInWorld * dt;
      y = y + yDotInWorld * dt;
      
      System.out.println("x = " + x + ", y = " + y);
      
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Vector2D me = new Vector2D(x, y);

      Vector2D meToMouse = new Vector2D(mouse);
      meToMouse.sub(me);
      meToMouse.normalize();

      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      double cross = headingVector.cross(meToMouse);
      double dot = headingVector.dot(meToMouse);

      double turnRate = 4.0 * cross;
      double acceleration = 4.0 * dot;

      return new double[] {acceleration, turnRate};
   }

   @Override
   public boolean getDropFlag()
   {
      return ((x > 8.0) && (y > 8.0));
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> vectorToInBodyFrameAndIdOfClosestFlag)
   {      
      Point2D vectorToFlag = vectorToInBodyFrameAndIdOfClosestFlag.getLeft();
      Integer flagId = vectorToInBodyFrameAndIdOfClosestFlag.getRight();
//      System.out.println("flag " + flagId + " is at" + vectorToFlag + ", in body frame");
   }

   @Override
   public void senseDroppedFlag(int id)
   {      
   }

   @Override
   public void sensePickedUpFlag(int id)
   {      
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {      
   }

   @Override
   public void senseWallRangeInBodyFrame(Vector2D vectorToWallInBodyFrame, double wallDistance)
   {
//      System.out.println("Distance to wall = " + wallDistance);      
   }

   @Override
   public void senseHitWall()
   {      
   }

}
