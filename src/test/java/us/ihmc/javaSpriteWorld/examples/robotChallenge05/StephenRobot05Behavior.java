package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

public class StephenRobot05Behavior implements Robot05Behavior
{
   @Override
   public void senseVelocity(double velocity)
   {

   }

   @Override
   public void senseHeading(double heading)
   {

   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {

   }

   @Override
   public void senseWallRangeInBodyFrame(Vector2D vectorToWallInBodyFrame, double wallDistance)
   {

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
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {

   }

   @Override
   public void senseDroppedFlag(int flagId)
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
   public void senseHitWall()
   {      
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      return new double[0];
   }

   @Override
   public boolean getDropFlag()
   {
      return false;
   }

}
