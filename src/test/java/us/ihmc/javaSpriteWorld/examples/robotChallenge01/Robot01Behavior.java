package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public interface Robot01Behavior
{
   public abstract void senseGlobalLocation(double x, double y);

   public abstract void senseVelocity(double velocity);

   public abstract void senseHeading(double heading);

   public abstract void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationAndVelocityOfAllFood);

   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);

   public abstract void senseDroppedFlag(int id);

   public abstract void sensePickedUpFlag(int id);
   
   public abstract void senseDeliveredFlag(int flagId);

   public abstract void senseHitWall();

   double[] getXYVelocity();

}
