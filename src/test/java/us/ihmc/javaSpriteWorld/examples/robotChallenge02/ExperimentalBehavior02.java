package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

public class ExperimentalBehavior02 implements Robot02Behavior
{
   private static final double minSpeed = 0.5;
   private static final double maxSpeed = 3.0;
   private static final double chaseNearestFoodDeadband = 3.0;
   private static final double turnInPlaceTowardsFoodThreshold = Math.toRadians(45.0);
   private static final double accelerationPerMeterAway = 1.0;
   private static final double turnRateMagnitude = 3.5;

   private static final double kLookAhead = 0.8;

   private double xPosition, yPosition;
   private double velocity, heading;

   private ArrayList<Pair<Vector2D, Vector2D>> locationAndVelocityOfAllFood;
   private final double[] accelerationAndTurnRate = new double[2];

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

      computeNearestFood();
      computeFoodCentroid();

      double distanceToClosestFood = EuclidCoreTools.norm(xPosition - nearestFood.getX(), yPosition - nearestFood.getY());
      if (checkNearBounds())
      {
         // sub behavior 0: avoid bounds
         targetPosition.set(5.0, 5.0);
      }
      else if (distanceToClosestFood < chaseNearestFoodDeadband)
      {
         // sub behavior 1: chase nearest food
         targetPosition.set(nearestFood);
      }
      else
      {
         // sub behavior 2: chase food centroid
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

      return accelerationAndTurnRate;
   }

   private void computeNearestFood()
   {
      double closestFoodDistance = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         Vector2D food = locationAndVelocityOfAllFood.get(i).getKey();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
            closestIndex = i;
         }
      }

      nearestFood.set(locationAndVelocityOfAllFood.get(closestIndex).getKey());
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