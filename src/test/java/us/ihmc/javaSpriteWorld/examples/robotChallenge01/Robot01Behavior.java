package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot01Behavior
{
   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationAndVelocityOfAllFood);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void droppedFlag(int id);

   public abstract void pickedUpFlag(int id);
   
   public abstract void deliveredFlag(int flagId);

   double[] getXYVelocity();
}
