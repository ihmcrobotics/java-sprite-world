package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;

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

   int currentFlagId = 0;
   boolean carrying = false;

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
      meToCenter.scale(0.5 * Math.pow(distanceToCenter, 1.2));

      Vector2D predatorRepulsion = new Vector2D();
      for (Pair<Vector2D, Vector2D> predator : locationOfAllPredators)
      {
         Vector2D predatorToMe = new Vector2D(me);
         predatorToMe.sub(predator.getLeft());
         double distance = predatorToMe.length();
         predatorToMe.normalize();
         predatorToMe.scale(5.0 / (distance * distance));
         predatorRepulsion.add(predatorToMe);
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      Vector2D foodAttraction = new Vector2D();
      for (Pair<Vector2D, Vector2D> food : locationOfAllFood)
      {
         Vector2D meToFood = new Vector2D(food.getLeft());
         meToFood.sub(me);
         double distance = meToFood.length();
         meToFood.normalize();
         meToFood.scale(1.2 / Math.pow(distance, 1.5));
         foodAttraction.add(meToFood);
      }

      Point2D goal = new Point2D(9.0, 9.0);
      if (carrying && new Point2D(me).distance(goal) < 1.0)
      {
               LogTools.info("GOAL {}", currentFlagId);
         carrying = false;
         currentFlagId++;

         if (currentFlagId >= locationAndIdsOfAllFlags.size())
         {
            currentFlagId = 0;
         }
      }

      Vector2D flagField = new Vector2D();
      for (int i = 0; i < locationAndIdsOfAllFlags.size(); i++)
      {
         Pair<Vector2D, Integer> flag = locationAndIdsOfAllFlags.get(i);

         if (i == currentFlagId)
         {
            if (!carrying && new Point2D(me).distance(new Point2D(flag.getLeft())) < 0.2)
            {
               LogTools.info("Carrying 1");
               carrying = true;
            }

            if (carrying)
            {
               Vector2D meToGoal = new Vector2D(goal);
               meToGoal.sub(me);
               double length = meToGoal.length();
               meToGoal.scale(2.0 / Math.pow(length, 1.5));
               flagField.add(meToGoal);
            }
            else
            {
               Vector2D meToNextFlag = new Vector2D(flag.getLeft());
               meToNextFlag.sub(me);
               double length = meToNextFlag.length();
               meToNextFlag.scale(2.0 / Math.pow(length, 1.5));
               flagField.add(meToNextFlag);
            }
         }
         else
         {
            Vector2D inactiveFlagToMe = new Vector2D(me);
            inactiveFlagToMe.sub(flag.getLeft());
            double length = inactiveFlagToMe.length();
            inactiveFlagToMe.scale(2.0 / Math.pow(length, 1.5));
            flagField.add(inactiveFlagToMe);
         }
      }

      Vector2D attractionVector = new Vector2D();
//      attractionVector.add(meToMouse);
      attractionVector.add(meToCenter);
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);
      attractionVector.add(flagField);

      double desiredSpeed = attractionVector.length();

      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attractionVector.getX(),
                                                                                    attractionVector.getY());

      double acceleration = (1.0 * (desiredSpeed - velocity));

      double angularVelocity = (velocity - lastVelocity) / 0.01;
      double turnRate = (2.0 * angleToAttraction) + (-0.5 * angularVelocity);
      lastVelocity = velocity;

      return new double[] {acceleration, turnRate};
   }

   double lastVelocity = 0.0;

   @Override
   public boolean getDropFlag()
   {
      return ((x > 8.0) && (y > 8.0));
   }
}
