package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.ArrayList;
import java.util.Arrays;

public class StephensRobot04Behavior implements Robot04Behavior
{
   // robot state
   private double xPosition;
   private double yPosition;
   private double velocity;
   private double heading;
   private final Vector2D xyVelocity = new Vector2D();

   // environment state
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators;
   private ArrayList<Pair<Vector2D, Integer>> locationAndIdsOfAllFlags;
   private final Point2D nearestFood = new Point2D();
   private final Point2D nearestPredator = new Point2D();

   // behavior parameters
   private final Vector2D targetPosition = new Vector2D();
   private final double[] accelerationAndTurnRate = new double[2];
   private final double turnRateMagnitude = 3.5;
   private final double foodProximityThreshold = 2.0;
   private final double predatorProximityThreshold = 2.0;

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
   public void senseGlobalLocation(double x, double y)
   {
      this.xPosition = x;
      this.yPosition = y;
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
   public double[] getAccelerationAndTurnRate()
   {
      if (locationOfAllFood == null || locationOfAllPredators == null || locationAndIdsOfAllFlags == null)
      {
         return new double[2];
      }

      calculateXYVelocity();

      if (checkIfNearWall())
      {
         targetPosition.set(5.0, 5.0);
         System.out.println("WALL");
         return simpleTurnTowardsGoalBehavior(nearestFood);
      }
      else if (checkIfNearPredator())
      {
//         Vector2D predatorToRobot = new Vector2D(nearestPredator);
//         predatorToRobot.sub(xPosition, yPosition);
//
//         Vector2D perpendicularVector1 = new Vector2D(-predatorToRobot.getY(), predatorToRobot.getX());
//         Vector2D perpendicularVector2 = new Vector2D(predatorToRobot.getY(), -predatorToRobot.getX());
//         double angle1 = Math.abs(perpendicularVector1.angle(xyVelocity));
//         double angle2 = Math.abs(perpendicularVector2.angle(xyVelocity));
//         Vector2D targetVector = angle1 < angle2 ? perpendicularVector1 : perpendicularVector2;
         System.out.println("PRED");

         double turnRate = calculatePredatorAction();

         double cruisingVelocity = 1.0;
         accelerationAndTurnRate[0] = 0.5 * (cruisingVelocity - velocity);
         accelerationAndTurnRate[1] = turnRate;
         return accelerationAndTurnRate;
      }
      else if (checkIfNearFood())
      {
         targetPosition.set(nearestFood);
         System.out.println("FOOD");
         return simpleTurnTowardsGoalBehavior(nearestFood);
      }

      return new double[2];
   }

   private double[] simpleTurnTowardsGoalBehavior(Tuple2DReadOnly targetPosition)
   {
      double[] accelerationAndTurnRate = new double[2];

      Vector2D toTargetPosition = new Vector2D(targetPosition);
      toTargetPosition.sub(xPosition, yPosition);

      double targetHeading = headingFromVector(toTargetPosition);
      double angleToTargetPosition = EuclidCoreTools.angleDifferenceMinusPiToPi(targetHeading, heading);
      double turnInPlaceThreshold = Math.toRadians(45.0);
      if (Math.abs(angleToTargetPosition) < turnInPlaceThreshold)
      {
         double cruisingVelocity = 1.0;
         accelerationAndTurnRate[0] = 0.5 * (cruisingVelocity - velocity);
         accelerationAndTurnRate[1] = turnRateMagnitude * (angleToTargetPosition / turnInPlaceThreshold);
      }
      else
      {
         accelerationAndTurnRate[0] = - velocity;
         accelerationAndTurnRate[1] = turnRateMagnitude * Math.signum(angleToTargetPosition);
      }

      return accelerationAndTurnRate;
   }

   private void calculateXYVelocity()
   {
      xyVelocity.setX(- velocity * Math.sin(heading));
      xyVelocity.setY(velocity * Math.cos(heading));
   }

   private static double headingFromVector(Vector2D vector)
   {
      return headingFromVector(vector.getX(), vector.getY());
   }

   private static double headingFromVector(double x, double y)
   {
      return Math.atan2(-x, y);
   }

   private boolean checkIfNearWall()
   {
      double wallProximitySensorLength = 1.0;

      for (int i = 0; i < 2; i++)
      {
         double dTheta = Math.toRadians(10.0);
         double headingToCheck = heading + dTheta;

         double wallProximitySensorDx = - wallProximitySensorLength * Math.sin(headingToCheck);
         double wallProximitySensorDy = wallProximitySensorLength * Math.cos(headingToCheck);

         double wallProximitySensorX = xPosition + wallProximitySensorDx;
         double wallProximitySensorY = yPosition + wallProximitySensorDy;

         if (wallProximitySensorX < 0.0 || wallProximitySensorX > 10.0 || wallProximitySensorY < 0.0 || wallProximitySensorY > 10.0)
         {
            return true;
         }
      }

      return false;
   }

   private boolean checkIfNearFood()
   {
      double closestFoodDistance = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationOfAllFood.size(); i++)
      {
         Vector2D food = locationOfAllFood.get(i).getKey();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
            closestIndex = i;
         }
      }

      nearestFood.set(locationOfAllFood.get(closestIndex).getKey());
      return nearestFood.distance(new Point2D(xPosition, yPosition)) < foodProximityThreshold;
   }

   private boolean checkIfNearPredator()
   {
      double closestPredator = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         Vector2D predator = locationOfAllPredators.get(i).getKey();
         double distance = EuclidCoreTools.norm(predator.getX() - xPosition, predator.getY() - yPosition);
         if (distance < closestPredator)
         {
            closestPredator = distance;
            closestIndex = i;
         }
      }

      nearestPredator.set(locationOfAllPredators.get(closestIndex).getKey());
      return nearestPredator.distance(new Point2D(xPosition, yPosition)) < predatorProximityThreshold;
   }

   private double calculatePredatorAction()
   {
      double turningAction = 0.0;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         Pair<Vector2D, Vector2D> predatorState = locationOfAllPredators.get(i);
         Vector2D predatorPosition = predatorState.getLeft();
         Vector2D predatorVelocity = predatorState.getRight();

         Vector2D predatorToRobot = new Vector2D(xPosition, yPosition);
         predatorToRobot.sub(predatorPosition);

         double maxAngleToConsider = Math.toRadians(70.0);
         if (Math.abs(predatorToRobot.angle(new Vector2D(predatorVelocity))) > maxAngleToConsider)
         {
            continue;
         }

         double distanceToPredator = predatorToRobot.length();
         predatorToRobot.normalize();

         Vector2D predatorRelativeVelocity = new Vector2D(predatorVelocity);
         predatorRelativeVelocity.sub(xyVelocity);

         Vector2D predatorRelativeVelocityTowardsRobot = new Vector2D(predatorRelativeVelocity);
         double alpha = predatorRelativeVelocityTowardsRobot.dot(predatorToRobot) / predatorRelativeVelocityTowardsRobot.length();
         predatorRelativeVelocityTowardsRobot.scale(alpha);

         double minimumDistanceThreshold = 0.5;
         double distanceMetric = distanceToPredator; // Math.max(distanceToPredator - kLookAhead * predatorRelativeVelocityTowardsRobot.length(), minimumDistanceThreshold);

         Vector2D perpendicularVector1 = new Vector2D(- predatorToRobot.getY(), predatorToRobot.getX());
         Vector2D perpendicularVector2 = new Vector2D(predatorToRobot.getY(), - predatorToRobot.getX());
         double angle1 = Math.abs(perpendicularVector1.angle(xyVelocity));
         double angle2 = Math.abs(perpendicularVector2.angle(xyVelocity));
         Vector2D targetVector = angle1 < angle2 ? perpendicularVector1 : perpendicularVector2;
         double targetHeading = headingFromVector(targetVector.getX(), targetVector.getY());
         turningAction += EuclidCoreTools.angleDifferenceMinusPiToPi(targetHeading, heading) / distanceMetric;
      }

      return turningAction;
   }

   @Override
   public boolean getDropFlag()
   {
      return false;
   }

   public static void main(String[] args)
   {
      System.out.println(headingFromVector(0.0, 1.0));
      System.out.println(headingFromVector(-1.0, 0.0));
      System.out.println(headingFromVector(0.0, -1.0));
      System.out.println(headingFromVector(1.0, 0.0));
   }
}
