package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

public class SimpleRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
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
      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      xDotInWorld = headingVector.getX() * velocity;
      yDotInWorld = headingVector.getY() * velocity;

      x = x + xDotInWorld * RobotChallenge01.dt;
      y = y + yDotInWorld * RobotChallenge01.dt;

      //      System.out.println("heading = " + heading + ", xDot = " + xDotInWorld + ", yDot = " + yDotInWorld + ", x = " + x + ", y = " + y);
      //      sleep(0.03);

      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Vector2D me = new Vector2D(x, y);

      Vector2D meToMouse = new Vector2D(mouse);
      meToMouse.sub(me);
      meToMouse.normalize();

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
//      Point2D vectorToFlag = vectorToInBodyFrameAndIdOfClosestFlag.getLeft();
//      Integer flagId = vectorToInBodyFrameAndIdOfClosestFlag.getRight();
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
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
//      System.out.println();
//      for (Pair<Vector2D, Double> rangePoint : vectorsAndDistancesToWallInBodyFrame)
//      {
//         System.out.println(rangePoint.getLeft() + ": " + rangePoint.getRight());
//      }
//      sleep(0.1);
   }

   @Override
   public void senseHitWall()
   {
   }

   private void sleep(double d)
   {
      try
      {
         Thread.sleep((long) (d * 1000));
      }
      catch (InterruptedException e)
      {
      }

   }

}
