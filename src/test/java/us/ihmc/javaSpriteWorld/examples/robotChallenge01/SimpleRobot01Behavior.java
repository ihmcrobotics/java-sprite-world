package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
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
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
   }

   @Override
   public double[] getXYVelocity()
   {
      return new double[] {3.0, 3.0};
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
   public void senseCarryingFlag(int flagId)
   {      
   }

   @Override
   public void senseHitWall()
   {      
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double time)
   {      
   }

   @Override
   public void reset()
   {      
   }

   @Override
   public void exit()
   {
   }

}
