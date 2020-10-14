package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot03Behavior
{
   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood);

   public abstract void sensePredators(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators);

   public abstract void senseDroppedFlag(int id);

   public abstract void sensePickedUpFlag(int id);

   public abstract void senseDeliveredFlag(int flagId);

   public abstract void senseHitWall();

   public abstract double[] getAccelerationAndTurnRate();

}
