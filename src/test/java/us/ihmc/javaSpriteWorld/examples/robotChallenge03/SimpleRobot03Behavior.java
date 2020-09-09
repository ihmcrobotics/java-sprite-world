package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;

public class SimpleRobot03Behavior implements Robot03Behavior
{
   public SimpleRobot03Behavior()
   {
   }

   @Override
   public void senseGlobalLocation(double x, double y)
   {
   }

   @Override
   public void senseVelocity(double velocity)
   {
   }

   @Override
   public void senseHeading(double heading)
   {
   }

   @Override
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood)
   {
   }

   @Override
   public void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators)
   {
   }

   @Override
   public void senseTreasure(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllTreasure)
   {
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      System.out.println("Mouse pressed at " + mousePressedX + ", " + mousePressedY);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      return new double[] {1.0, -0.3};
   }

}
