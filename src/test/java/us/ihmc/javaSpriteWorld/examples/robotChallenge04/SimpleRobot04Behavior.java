package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class SimpleRobot04Behavior implements Robot04Behavior
{
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double x = 0.0, y = 0.0;
   private double heading = 0.0;

   public SimpleRobot04Behavior()
   {
   }

   @Override
   public void senseGlobalLocation(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   @Override
   public void senseVelocity(double velocity)
   {
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = heading;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {      
      System.out.println("Key Pressed: " + keyPressed);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Vector2D me = new Vector2D(x, y);

      Vector2D meToMouse = new Vector2D(mouse);
      meToMouse.sub(me);
      meToMouse.normalize();

      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      double cross = headingVector.cross(meToMouse);
      double dot = headingVector.dot(meToMouse);

      double turnRate = 4.0 * cross;
      double acceleration = 4.0 * dot;

      return new double[] {acceleration, turnRate};
   }

   @Override
   public void senseFlags(ArrayList<Pair<Point2D, Integer>> locationAndIdsOfAllFlags)
   {
   }

   @Override
   public boolean getDropFlag()
   {
      return ((x > 8.0) && (y > 8.0));
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
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {      
   }

   @Override
   public void sensePredators(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
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
