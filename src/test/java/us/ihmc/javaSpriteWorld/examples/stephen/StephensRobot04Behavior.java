package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.robotChallenge04.Robot04Behavior;

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
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private ArrayList<Pair<Point2D, Integer>> locationAndIdsOfAllFlags;
   private final Point2D nearestFood = new Point2D();
   private final Point2D nearestPredator = new Point2D();
   private double nearestPredatorDistance = 0.0;
   private double nearestDistanceToFood = 0.0;

   // behavior parameters
   private final Point2D center = new Point2D(5.0, 5.0);

   private final double turnRateMagnitude = 3.5;
   private final double foodProximityThreshold = 4.0;
   private final double predatorProximityThreshold = 4.0;

   private final double[] wallAction = new double[2];
   private final double[] predatorAction = new double[2];
   private final double[] foodAction = new double[2];
   private final double[] flagAction = new double[2];
   private final double[] totalAction = new double[2];

   private double totalWeight;
   private final double kWall = 5.0;

   private boolean inFlagDropMode = false;
   private int flagIdToChase = 1;
   private boolean droppedFlagLastTick = false;
   private boolean hasAFlag = false;

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
   public void senseFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredators(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseFlags(ArrayList<Pair<Point2D, Integer>> locationAndIdsOfAllFlags)
   {
      this.locationAndIdsOfAllFlags = locationAndIdsOfAllFlags;
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
   public void senseKeyPressed(String keyPressed)
   {

   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      if (locationOfAllFood == null || locationOfAllPredators == null || locationAndIdsOfAllFlags == null)
      {
         return new double[2];
      }

      calculateXYVelocity();
      Arrays.fill(wallAction, 0.0);
      Arrays.fill(predatorAction, 0.0);
      Arrays.fill(foodAction, 0.0);
      Arrays.fill(flagAction, 0.0);
      Arrays.fill(totalAction, 0.0);
      totalWeight = 0.0;

      if (checkIfNearWall())
      {
         simpleTurnTowardsGoalBehavior(center, wallAction);
         registerAction(wallAction, kWall);
      }
      if (checkIfNearPredator())
      {
         double turnRate = calculatePredatorAction();

         double cruisingVelocity = 1.5;
         predatorAction[0] = 0.5 * (cruisingVelocity - velocity);
         predatorAction[1] = turnRate;

         double kPredator = Math.max(0.0, 4.0 - nearestPredatorDistance);
         registerAction(predatorAction, kPredator);
      }
      if (checkIfNearFood())
      {
         simpleTurnTowardsGoalBehavior(nearestFood, foodAction);

         double kFood = Math.max(0.0, 4.0 - nearestDistanceToFood);
         registerAction(foodAction, kFood);
      }

      if (droppedFlagLastTick)
      {
         flagIdToChase++;
         droppedFlagLastTick = false;
      }

      if (inFlagDropMode)
      {
         simpleTurnTowardsGoalBehavior(new Point2D(9.0, 9.0), flagAction);
      }
      else
      {
         simpleTurnTowardsGoalBehavior(locationAndIdsOfAllFlags.get(flagIdToChase - 1).getLeft(), flagAction);
      }

      double kFlag = Math.max(0.0, 4.0 - totalWeight);
      registerAction(flagAction, kFlag);

      totalAction[0] = totalAction[0] / totalWeight;
      totalAction[1] = totalAction[1] / totalWeight;

      return totalAction;
   }

   private void simpleTurnTowardsGoalBehavior(Tuple2DReadOnly targetPosition, double[] action)
   {
      Vector2D toTargetPosition = new Vector2D(targetPosition);
      toTargetPosition.sub(xPosition, yPosition);

      double targetHeading = headingFromVector(toTargetPosition);
      double angleToTargetPosition = EuclidCoreTools.angleDifferenceMinusPiToPi(targetHeading, heading);
      double turnInPlaceThreshold = Math.toRadians(45.0);
      if (Math.abs(angleToTargetPosition) < turnInPlaceThreshold)
      {
         double cruisingVelocity = 1.25;
         action[0] = 0.5 * (cruisingVelocity - velocity);
         action[1] = turnRateMagnitude * (angleToTargetPosition / turnInPlaceThreshold);
      }
      else
      {
         action[0] = - velocity;
         action[1] = turnRateMagnitude * Math.signum(angleToTargetPosition);
      }
   }

   private void registerAction(double[] action, double gain)
   {
      totalWeight += gain;
      totalAction[0] += gain * action[0];
      totalAction[1] += gain * action[1];
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
         Point2D food = locationOfAllFood.get(i).getMiddle();
         double distance = EuclidCoreTools.norm(food.getX() - xPosition, food.getY() - yPosition);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
            closestIndex = i;
         }
      }

      nearestFood.set(locationOfAllFood.get(closestIndex).getMiddle());
      nearestDistanceToFood = nearestFood.distance(new Point2D(xPosition, yPosition));
      return nearestDistanceToFood < foodProximityThreshold;
   }

   private boolean checkIfNearPredator()
   {
      double closestPredator = Double.MAX_VALUE;
      int closestIndex = -1;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         Point2D predator = locationOfAllPredators.get(i).getKey();
         double distance = EuclidCoreTools.norm(predator.getX() - xPosition, predator.getY() - yPosition);
         if (distance < closestPredator)
         {
            closestPredator = distance;
            closestIndex = i;
         }
      }

      nearestPredator.set(locationOfAllPredators.get(closestIndex).getKey());
      nearestPredatorDistance = nearestPredator.distance(new Point2D(xPosition, yPosition));
      return nearestPredatorDistance < predatorProximityThreshold;
   }

   private double calculatePredatorAction()
   {
      double turningAction = 0.0;

      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         Pair<Point2D, Vector2D> predatorState = locationOfAllPredators.get(i);
         Point2D predatorPosition = predatorState.getLeft();
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
         double kLookAhead = 0.35;

         double distanceMetric = Math.max(distanceToPredator - kLookAhead * predatorRelativeVelocityTowardsRobot.length(), minimumDistanceThreshold);

         Vector2D perpendicularVector1 = new Vector2D(- predatorToRobot.getY(), predatorToRobot.getX());
         Vector2D perpendicularVector2 = new Vector2D(predatorToRobot.getY(), - predatorToRobot.getX());
         double angle1 = Math.abs(perpendicularVector1.angle(xyVelocity));
         double angle2 = Math.abs(perpendicularVector2.angle(xyVelocity));
         Vector2D targetVector = angle1 < angle2 ? perpendicularVector1 : perpendicularVector2;
         double targetHeading = headingFromVector(targetVector.getX(), targetVector.getY());
         turningAction += 1.5 * EuclidCoreTools.angleDifferenceMinusPiToPi(targetHeading, heading) / distanceMetric;
      }

      return turningAction;
   }

   @Override
   public boolean getDropFlag()
   {
      if (!hasAFlag)
         return false;
      if (xPosition > 8.0 && yPosition > 8.0)
      {
         droppedFlagLastTick = true;
         return true;
      }

      return false;
   }

   public static void main(String[] args)
   {
      System.out.println(headingFromVector(0.0, 1.0));
      System.out.println(headingFromVector(-1.0, 0.0));
      System.out.println(headingFromVector(0.0, -1.0));
      System.out.println(headingFromVector(1.0, 0.0));
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
}
