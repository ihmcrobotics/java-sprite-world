package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

/**
 * XY axis origin is bottom left
 * X is up
 * Y is right
 */
public class DuncansRobot02Behavior implements Robot02Behavior
{
   private ArrayList<Food> foods;
   private double correctedHeading;
   private double velocityMagnitude;
   private Point2D robotPosition;
   private Vector2D robotVelocity;


   class Food
   {
      private final Point2D position;
      private final Vector2D velocity;

      public Food(Point2D position, Vector2D velocity)
      {
         this.position = position;
         this.velocity = velocity;
      }

      public Point2D getPosition()
      {
         return position;
      }

      public Vector2D getVelocity()
      {
         return velocity;
      }
   }

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
      this.correctedHeading = heading - Math.PI / 2.0;
   }

   @Override
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      foods = new ArrayList<>();
      for (Triple<Integer, Point2D, Vector2D> vector2DVector2DPair : locationOfAllFood)
      {
         foods.add(new Food(new Point2D(vector2DVector2DPair.getMiddle()), new Vector2D(vector2DVector2DPair.getRight())));
      }
   }
   
   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      System.out.println("Mouse pressed at " + mousePressedX + ", " + mousePressedY);
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {

   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      double acceleration = 0.0;
      double turnRate = 0.0;

      if (!foods.isEmpty())
      {
         robotVelocity = new Vector2D(Math.cos(correctedHeading), -Math.sin(correctedHeading));
         robotVelocity.normalize();
         robotVelocity.scale(velocityMagnitude);

         double closestDistance = foods.get(0).getPosition().distance(robotPosition);
         int targetIndex = 0;
         Food closestFood = foods.get(0);
         Point2D foodCentroid = new Point2D();
         for (int i = 1; i < foods.size(); i++)
         {
            Food food = foods.get(i);
            double distance = food.getPosition().distance(robotPosition);
            if (distance < closestDistance)
            {
               closestFood = food;
               closestDistance = distance;
               targetIndex = i;
            }
            foodCentroid.add(food.getPosition());
         }
         foodCentroid.setX(foodCentroid.getX() / foods.size());
         foodCentroid.setY(foodCentroid.getY() / foods.size());

         Point2D target;
         double desiredSpeed;
//         if (closestDistance > 2.0)
//         {
//            target = foodCentroid;
//            desiredSpeed = 0.5;
//         }
//         else
         {
            target = closestFood.getPosition();
            desiredSpeed = 1.5;
         }

         Vector2D robotToTarget = new Vector2D(target);
         robotToTarget.sub(robotPosition);

         double desiredHeading = EuclidGeometryTools.angleFromXForwardToVector2D(robotToTarget);
         double targetHeadingCorrection = EuclidCoreTools.angleDifferenceMinusPiToPi(desiredHeading, correctedHeading);

         acceleration = robotVelocity.length() > desiredSpeed ? -0.5 : 0.5;
//         if (Math.toDegrees(Math.abs(targetHeadingCorrection)) > 90.0) // if robot facing away from food, go backwards
//            acceleration = -acceleration;
         turnRate = targetHeadingCorrection > 0 ? -1.5 : 1.5;
      }

      return new double[] {acceleration, turnRate};
   }

   @Override
   public void senseDroppedFlag(int id)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub
      
   }
}
