package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class SimpleRobot02Behavior implements Robot02Behavior
{
   public SimpleRobot02Behavior()
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
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
   }
   
   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      System.out.println("Mouse pressed at " + mousePressedX + ", " + mousePressedY);
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {      
      System.out.println("Key Pressed: " + keyPressed);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      return new double[] {1.0, -0.3};
   }

   @Override
   public void senseDroppedFlag(int id)
   {      
   }

   @Override
   public void sensePickedUpFlag(int id)
   {      
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {      
   }

   @Override
   public void senseHitWall()
   {      
   }

}
