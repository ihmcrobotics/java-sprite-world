package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class ExperimentalBehavior01 implements Robot01Behavior
{
   private static final double minSpeed = 0.5;
   private static final double maxSpeed = 3.0;
   private static final double chaseNearestFoodDeadband = 2.0;

   private double xPosition, yPosition;
   private double velocity, heading;

   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationAndVelocityOfAllFood;
   private final double[] xyVelocity = new double[2];

   private final Vector2D nearestFood = new Vector2D();
   private final Vector2D foodCentroid = new Vector2D();

   @Override
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationAndVelocityOfAllFood)
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
   public void senseKeyPressed(String keyPressed)
   {
   }

   @Override
   public double[] getXYVelocity()
   {
      if (locationAndVelocityOfAllFood == null)
      {
         return new double[2];
      }

      computeNearestFood();
      computeFoodCentroid();

      Vector2D desiredVelocity = new Vector2D(nearestFood);
      desiredVelocity.sub(xPosition, yPosition);
      double distanceFromFood = desiredVelocity.length();

      if (distanceFromFood > chaseNearestFoodDeadband)
      {
         desiredVelocity.set(foodCentroid.getX() - xPosition, foodCentroid.getY() - yPosition);
         desiredVelocity.normalize();
         desiredVelocity.scale(minSpeed);
      }
      else
      {
         double velocity = maxSpeed - (distanceFromFood / chaseNearestFoodDeadband) * (maxSpeed - minSpeed);
         desiredVelocity.scale(velocity);
      }

      xyVelocity[0] = desiredVelocity.getX();
      xyVelocity[1] = desiredVelocity.getY();

      return xyVelocity;
   }

   private void computeNearestFood()
   {
      double closestFoodDistance = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         Point2D food = locationAndVelocityOfAllFood.get(i).getMiddle();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
            closestIndex = i;
         }
      }

      nearestFood.set(locationAndVelocityOfAllFood.get(closestIndex).getMiddle());
   }

   private void computeFoodCentroid()
   {
      foodCentroid.setToZero();
      double totalWeight = 0.0;

      for (int i = 0; i < locationAndVelocityOfAllFood.size(); i++)
      {
         Point2D food = locationAndVelocityOfAllFood.get(i).getMiddle();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         double weight = 1.0 / distance;

         Vector2D weightedFood = new Vector2D(food);
         weightedFood.scale(weight);
         foodCentroid.add(weightedFood);
         totalWeight += weight;
      }

      foodCentroid.scale(1.0 / totalWeight);
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
