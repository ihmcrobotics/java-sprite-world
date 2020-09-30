package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot05Behavior
{
   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame);

   public abstract void senseFoodInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood);

   public abstract void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators);

   public abstract void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag);

   public abstract void senseDroppedFlag(int flagId);

   public abstract void sensePickedUpFlag(int id);

   public abstract void senseDeliveredFlag(int flagId);

   public abstract void senseHitWall();

   public abstract double[] getAccelerationAndTurnRate();

   public abstract boolean getDropFlag();

}
