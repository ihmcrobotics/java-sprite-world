package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

public class DuncanRobot04Behavior implements Robot04Behavior
{
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double x = 0.0, y = 0.0;
   private double heading = 0.0;
   private double velocity;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators;
   private ArrayList<Pair<Vector2D, Integer>> locationAndIdsOfAllFlags;

   public DuncanRobot04Behavior()
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
      this.velocity = velocity;
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = heading;
   }

   @Override
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseFlags(ArrayList<Pair<Vector2D, Integer>> locationAndIdsOfAllFlags)
   {
      this.locationAndIdsOfAllFlags = locationAndIdsOfAllFlags;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Vector2D me = new Vector2D(x, y);

      Vector2D meToMouse = new Vector2D(mouse);
      meToMouse.sub(me);
      meToMouse.normalize();

      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = new Vector2D(center);
      meToCenter.sub(me);
      double distanceToCenter = meToCenter.length();
      meToCenter.normalize();
      meToCenter.scale(distanceToCenter * distanceToCenter);

      


      Vector2D attractionVector = meToCenter;

      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      double cross = headingVector.cross(attractionVector);
      double dot = headingVector.dot(attractionVector);

      double turnRate = 4.0 * cross;
      double acceleration = 4.0 * dot;

      return new double[] {acceleration, turnRate};
   }

   @Override
   public boolean getDropFlag()
   {
      return ((x > 8.0) && (y > 8.0));
   }
}
