package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

public class ExperimentalBehavior03 implements Robot03Behavior
{
   private final int minimumNumberOfTicksPerState = 60;

   private enum State
   {
      ESCAPE_BOUNDS,
      ESCAPE_PREDATOR,
      CHASE_FOOD,
      TURN_TORWARDS_MIDDLE
   }

   private State previousState;
   private int numberOfTicksInState;

   private static final double chaseNearestFoodDeadband = 2.5;
   private static final double escapeNearestPredatorProximity = 2.5;
   private static final double turnInPlaceTowardsFoodThreshold = Math.toRadians(45.0);
   private static final double accelerationPerMeterAway = 1.0;
   private static final double turnRateMagnitude = 3.5;

   private static final double kLookAhead = 0.8;

   private double xPosition, yPosition;
   private double velocity, heading;

   private ArrayList<Pair<Vector2D, Vector2D>> locationAndVelocityOfAllFood;
   private final double[] accelerationAndTurnRate = new double[2];

   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators;
   private final Vector2D nearestPredator = new Vector2D();

   private final Vector2D nearestFood = new Vector2D();
   private final Vector2D foodCentroid = new Vector2D();
   private final Vector2D targetPosition = new Vector2D();
   private final Vector2D lookAheadPosition = new Vector2D();

   @Override
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationAndVelocityOfAllFood)
   {
      this.locationAndVelocityOfAllFood = locationAndVelocityOfAllFood;
   }

   @Override
   public void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseTreasure(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllTreasure)
   {

   }

   @Override
   public void senseGlobalLocation(double x, double y)
   {
      this.xPosition = x;
      this.yPosition = y;
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
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {

   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      if (locationAndVelocityOfAllFood == null)
      {
         return new double[2];
      }

      computeNearestFood(xPosition, yPosition);
      computeFoodCentroid();
      computeNearestPredator();

      State desiredState = getDesiredState();

      if (previousState != null && desiredState != previousState)
      {
         if (numberOfTicksInState < minimumNumberOfTicksPerState)
         {
            desiredState = previousState;
            numberOfTicksInState++;
         }
         else
         {
            numberOfTicksInState = 0;
         }
      }
      else
      {
         numberOfTicksInState++;
      }

      switch (desiredState)
      {
         case ESCAPE_BOUNDS:
            // sub behavior 0: avoid bounds
            targetPosition.set(5.0, 5.0);
            break;
         case ESCAPE_PREDATOR:
            // sub behavior 1: avoid predator
            Vector2D vectorFromPredator = new Vector2D(xPosition, yPosition);
            vectorFromPredator.sub(nearestPredator);

            double turnAwayAngle = Math.toRadians(30.0);
            Orientation2D orientationTransform = new Orientation2D();
            orientationTransform.setYaw(turnAwayAngle);
            orientationTransform.transform(vectorFromPredator);

            targetPosition.set(xPosition, yPosition);
            targetPosition.add(vectorFromPredator);
            break;
         case CHASE_FOOD:
            // sub behavior 2: chase nearest food
            targetPosition.set(nearestFood);
            break;
         case TURN_TORWARDS_MIDDLE:
            // sub behavior 3: chase food centroid
            targetPosition.set(foodCentroid);
      }

      targetPosition.sub(xPosition, yPosition);

      double targetHeading = - Math.atan2(targetPosition.getX(), targetPosition.getY());
      double angleToTargetPosition = EuclidCoreTools.angleDifferenceMinusPiToPi(targetHeading, heading);
      if (Math.abs(angleToTargetPosition) < turnInPlaceTowardsFoodThreshold)
      {
         accelerationAndTurnRate[0] = targetPosition.length() * accelerationPerMeterAway;
         accelerationAndTurnRate[1] = turnRateMagnitude * (angleToTargetPosition / turnInPlaceTowardsFoodThreshold);
      }
      else
      {
         accelerationAndTurnRate[0] = - velocity;
         accelerationAndTurnRate[1] = turnRateMagnitude * Math.signum(angleToTargetPosition);
      }

      previousState = desiredState;
      return accelerationAndTurnRate;
   }

   private State getDesiredState()
   {
      double distanceToClosestFood = EuclidCoreTools.norm(xPosition - nearestFood.getX(), yPosition - nearestFood.getY());
      double distanceToClosestPredator = EuclidCoreTools.norm(xPosition - nearestPredator.getX(), yPosition - nearestPredator.getY());

      if (checkNearBounds())
      {
         return State.ESCAPE_BOUNDS;
      }
      else if (distanceToClosestPredator < escapeNearestPredatorProximity)
      {
         return State.ESCAPE_PREDATOR;

      }
      else if (distanceToClosestFood < chaseNearestFoodDeadband)
      {
         return State.CHASE_FOOD;

      }
      else
      {
         return State.TURN_TORWARDS_MIDDLE;
      }
   }

   private void computeNearestFood(double x, double y)
   {
      double closestFoodDistance = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         Vector2D food = locationAndVelocityOfAllFood.get(i).getKey();
         double distance = EuclidCoreTools.norm(food.getX() - x, food.getY() - y);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
            closestIndex = i;
         }
      }

      nearestFood.set(locationAndVelocityOfAllFood.get(closestIndex).getKey());
   }

   private void computeNearestPredator()
   {
      double closestPredatorDistance = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         Vector2D predator = locationOfAllPredators.get(i).getKey();
         double distance = EuclidCoreTools.norm(predator.getX() - xPosition, predator.getY() - yPosition);
         if (distance < closestPredatorDistance)
         {
            closestPredatorDistance = distance;
            closestIndex = i;
         }
      }

      nearestPredator.set(locationOfAllPredators.get(closestIndex).getKey());
   }

   private void computeFoodCentroid()
   {
      foodCentroid.setToZero();
      double totalWeight = 0.0;

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         Vector2D food = locationAndVelocityOfAllFood.get(i).getKey();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         double weight = 1.0 / distance;

         Vector2D weightedFood = new Vector2D(food);
         weightedFood.scale(weight);
         foodCentroid.add(weightedFood);
         totalWeight += weight;
      }

      foodCentroid.scale(1.0 / totalWeight);
   }

   private boolean checkNearBounds()
   {
      lookAheadPosition.set(xPosition, yPosition);
      Vector2D xyVelocity = new Vector2D();
      xyVelocity.setX(- Math.sin(heading) * velocity);
      xyVelocity.setY(Math.cos(heading) * velocity);
      Vector2D lookAheadVector = new Vector2D(xyVelocity);
      lookAheadVector.scale(kLookAhead);
      lookAheadPosition.add(lookAheadVector);

      boolean nearBounds = lookAheadPosition.getX() < 0.0 || lookAheadPosition.getX() > 10.0 || lookAheadPosition.getY() < 0.0 || lookAheadPosition.getY() > 10.0;
      if (nearBounds)
      {
         System.out.println("near bounds detected");
      }

      return nearBounds;
   }
}