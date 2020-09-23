package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.Arrays;

public class StephenRobot05Behavior implements Robot05Behavior
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
   private Pair<Point2D, Integer> locationAndIdOfClosestFlag;

   // behavior parameters
   private final double[] wallForce = new double[2];
   private final double[] foodForce = new double[2];
   private final double[] predatorForce = new double[2];
   private final double[] flagAttractorForce = new double[2];
   private final double[] flagRepulsiveForce = new double[2];

   private final double[] totalForce = new double[2];
   private final double[] totalAction = new double[2];

   private final double maxForceWall = 3.0;
   private final double maxForceFood = 1.0;
   private final double maxForcePredator = 3.0;
   private final double maxForceFlagRepulsive = 4.0;

   private final double constantForceFlagAttractor = 2.0;

   private final double expWall = 2.6;
   private final double expFood = 2.0;
   private final double expPredator = 2.3;
   private final double expFlagRepulsive = 2.5;

   // debug variables
   private double percentageWall;
   private double percentageFood;
   private double percentagePredator;
   private double percentageFlagAttractor;
   private double percentageFlagRepulsive;

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
   public double[] getAccelerationAndTurnRate()
   {
      if (locationOfAllFood == null || locationOfAllPredators == null || locationAndIdOfClosestFlag == null)
      {
         return new double[2];
      }

      calculateXYVelocity();
      Arrays.fill(wallForce, 0.0);
      Arrays.fill(foodForce, 0.0);
      Arrays.fill(predatorForce, 0.0);
      Arrays.fill(flagAttractorForce, 0.0);
      Arrays.fill(flagRepulsiveForce, 0.0);

      if (droppedFlagLastTick)
      {
         flagIdToChase++;
         droppedFlagLastTick = false;
      }

      // wall
      computeForce(wallForce, new Point2D(xPosition, 0.0), maxForceWall, expWall, false);
      computeForce(wallForce, new Point2D(xPosition, 10.0), maxForceWall, expWall, false);
      computeForce(wallForce, new Point2D(0.0, yPosition), maxForceWall, expWall, false);
      computeForce(wallForce, new Point2D(10.0, yPosition), maxForceWall, expWall, false);

      // predators
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         computePredatorForce(predatorForce, locationOfAllPredators.get(i).getKey(), locationOfAllPredators.get(i).getValue(), maxForcePredator, expPredator);
      }

      // food
      for (int i = 0; i < locationOfAllFood.size(); i++)
      {
         computeForce(foodForce, locationOfAllFood.get(i).getKey(), maxForceFood, expFood, true);
      }

      // flag attract
      Integer closestFlagID = locationAndIdOfClosestFlag.getValue();
      boolean collectClosestFlag = closestFlagID == flagIdToChase;

      if (inFlagDropMode)
      {
         computeFlagAttractForce(flagAttractorForce, new Point2D(9.0, 9.0), constantForceFlagAttractor);

         if (collectClosestFlag)
         {
            LogTools.info("shouldn't get here...");
         }
         else
         {
            computeForce(flagRepulsiveForce, locationAndIdOfClosestFlag.getKey(), maxForceFlagRepulsive, expFlagRepulsive, false);
         }
      }
      else
      {
         if (collectClosestFlag)
         {
            computeFlagAttractForce(flagAttractorForce, locationAndIdOfClosestFlag.getKey(), constantForceFlagAttractor);
         }
         else
         {
            computeForce(flagRepulsiveForce, locationAndIdOfClosestFlag.getKey(), maxForceFlagRepulsive, expFlagRepulsive, false);
         }
      }

      for (int i = 0; i < 2; i++)
      {
         totalForce[i] = wallForce[i] + predatorForce[i] + foodForce[i] + flagAttractorForce[i] + flagRepulsiveForce[i];
      }

      double totalForceMagnitude = EuclidCoreTools.norm(totalForce[0], totalForce[1]);
      percentageWall = EuclidCoreTools.norm(wallForce[0], wallForce[1]) / totalForceMagnitude;
      percentageFood = EuclidCoreTools.norm(foodForce[0], foodForce[1]) / totalForceMagnitude;
      percentagePredator = EuclidCoreTools.norm(predatorForce[0], predatorForce[1]) / totalForceMagnitude;
      percentageFlagAttractor = EuclidCoreTools.norm(flagAttractorForce[0], flagAttractorForce[1]) / totalForceMagnitude;
      percentageFlagRepulsive = EuclidCoreTools.norm(flagRepulsiveForce[0], flagRepulsiveForce[1]) / totalForceMagnitude;

      // convert to acceleration and steering action
      double desiredHeading = headingFromVector(totalForce[0], totalForce[1]);
      double deltaDesiredHeading = EuclidCoreTools.angleDifferenceMinusPiToPi(desiredHeading, heading);

      double velocityWhenAligned = 2.0;
      double targetVelocity;

      if (Math.abs(deltaDesiredHeading) < 0.5 * Math.PI)
         targetVelocity = velocityWhenAligned - (velocityWhenAligned / (0.5 * Math.PI)) * Math.abs(deltaDesiredHeading);
      else
         targetVelocity = 0.0;

      double kAcceleration = 3.0;
      double accelerationAction = kAcceleration * (targetVelocity - velocity);

      double kTurn = 1.0;
      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = accelerationAction;
      totalAction[1] = turningAction;

      return totalAction;
   }

   private void calculateXYVelocity()
   {
      xyVelocity.setX(- velocity * Math.sin(heading));
      xyVelocity.setY(velocity * Math.cos(heading));
   }

   private void computeForce(double[] scaledForceToSet, Tuple2DReadOnly point, double maxForce, double exp, boolean attract)
   {
      Vector2D force = new Vector2D(xPosition, yPosition);
      force.sub(point);

      if (attract)
      {
         force.scale(-1.0);
      }

      double distance = EuclidCoreTools.norm(point.getX() - xPosition, point.getY() - yPosition);

      force.normalize();
      force.scale(maxForce * Math.pow(distance, -exp));

      scaledForceToSet[0] += force.getX();
      scaledForceToSet[1] += force.getY();
   }

   private void computePredatorForce(double[] scaledForceToSet, Tuple2DReadOnly point, Tuple2DReadOnly velocity, double maxForce, double exp)
   {
      double kLookAhead = 0.3;
      Vector2D force = new Vector2D(point);
      force.add(kLookAhead * velocity.getX(), kLookAhead * velocity.getY());
      force.sub(xPosition + kLookAhead * this.xyVelocity.getX(), yPosition + kLookAhead * this.xyVelocity.getY());

      double distance = EuclidCoreTools.norm(point.getX() - xPosition, point.getY() - yPosition);

      force.normalize();
      force.scale(maxForce * Math.pow(distance, -exp));

      Vector2D perpVector1 = new Vector2D(- force.getY(), force.getX());
      Vector2D perpVector2 = new Vector2D(force.getY(), - force.getX());

      double deltaHeadingPerp1 = EuclidCoreTools.angleDifferenceMinusPiToPi(headingFromVector(perpVector1.getX(), perpVector1.getY()), heading);
      double deltaHeadingPerp2 = EuclidCoreTools.angleDifferenceMinusPiToPi(headingFromVector(perpVector2.getX(), perpVector2.getY()), heading);

      if (Math.abs(deltaHeadingPerp1) < Math.abs(deltaHeadingPerp2))
      {
         force.set(perpVector1);
      }
      else
      {
         force.set(perpVector2);
      }

      scaledForceToSet[0] += force.getX();
      scaledForceToSet[1] += force.getY();
   }

   private void computeFlagAttractForce(double[] scaledForceToSet, Tuple2DReadOnly point, double magnitude)
   {
      Vector2D force = new Vector2D(point);
      force.sub(xPosition, yPosition);

      force.normalize();
      force.scale(magnitude);

      scaledForceToSet[0] += force.getX();
      scaledForceToSet[1] += force.getY();
   }

   private static double headingFromVector(Vector2D vector)
   {
      return headingFromVector(vector.getX(), vector.getY());
   }

   private static double headingFromVector(double x, double y)
   {
      return Math.atan2(-x, y);
   }

   private boolean inFlagDropMode = false;
   private int flagIdToChase = 1;
   private boolean droppedFlagLastTick = false;

   @Override
   public boolean getDropFlag()
   {
      if (!inFlagDropMode)
         return false;

      return (xPosition > 8.0 && yPosition > 8.0);
   }

   @Override
   public void senseClosestFlag(Pair<Point2D, Integer> locationAndIdsOfClosestFlag)
   {
      this.locationAndIdOfClosestFlag = locationAndIdsOfClosestFlag;
   }

   @Override
   public void droppedFlag(int id)
   {
      inFlagDropMode = false;
   }

   @Override
   public void pickedUpFlag(int id)
   {
      if (flagIdToChase == id)
      {
         inFlagDropMode = true;
      }
      else if (inFlagDropMode)
      {
         inFlagDropMode = false;
      }
   }

   @Override
   public void deliveredFlag(int flagId)
   {
      flagIdToChase++;
      inFlagDropMode = false;
   }

   public static void main(String[] args)
   {
      System.out.println(headingFromVector(0.0, 1.0));
      System.out.println(headingFromVector(-1.0, 0.0));
      System.out.println(headingFromVector(0.0, -1.0));
      System.out.println(headingFromVector(1.0, 0.0));
   }
}
