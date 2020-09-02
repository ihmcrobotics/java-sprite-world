package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Vector2D;

public class SimpleRobot01Behavior implements Robot01Behavior
{
   public SimpleRobot01Behavior()
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
   public void senseFood(ArrayList<Vector2D> locationOfAllFood)
   {
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      double acceleration = 0.5;
      double turnRate = -0.2;
      return new double[] {acceleration, turnRate};
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      System.out.println("Mouse pressed at " + mousePressedX + ", " + mousePressedY);
   }

}
