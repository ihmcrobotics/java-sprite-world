package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot04Behavior
{
   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood);

   public abstract void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators);

   public abstract void senseFlags(ArrayList<Pair<Vector2D, Integer>> locationAndIdsOfAllFlags);

   public abstract double[] getAccelerationAndTurnRate();

   public abstract boolean getDropFlag();

}
