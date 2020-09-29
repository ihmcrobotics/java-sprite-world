package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot02Behavior
{
   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseFood(ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood);

   public abstract void droppedFlag(int id);

   public abstract void pickedUpFlag(int id);

   public abstract void deliveredFlag(int flagId);

   public abstract void hitWall();

   public abstract double[] getAccelerationAndTurnRate();

}
