package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

/**
 * XY axis origin is bottom left
 * X is up
 * Y is right
 */
public class DuncansRobot02Behavior implements Robot02Behavior
{
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood;
   private double headingCounterclockwiseFromUp;
   private double velocityMagnitude;
   private Point2D robotPosition;
   private Vector2D robotVelocity;

   public DuncansRobot02Behavior()
   {
   }

   @Override
   public void senseGlobalLocation(double x, double y)
   {
      robotPosition = new Point2D(x, y);
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocityMagnitude = velocity;
   }

   @Override
   public void senseHeading(double heading)
   {
      this.headingCounterclockwiseFromUp = heading;
   }

   @Override
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }
   
   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      System.out.println("Mouse pressed at " + mousePressedX + ", " + mousePressedY);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      robotVelocity = new Vector2D(Math.cos(headingCounterclockwiseFromUp), -Math.sin(headingCounterclockwiseFromUp));
      robotVelocity.normalize();
      robotVelocity.scale(velocityMagnitude);



      double acceleration = 5.0;
      double turnRate = -0.3;
      return new double[] {acceleration, turnRate};
   }

}
