package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot01Behavior
{
   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseFood(ArrayList<Vector2D> locationOfAllFood);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract double[] getAccelerationAndTurnRate();

}
