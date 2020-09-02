package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot02Behavior
{
   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood);

   public abstract double[] getAccelerationAndTurnRate();

}
