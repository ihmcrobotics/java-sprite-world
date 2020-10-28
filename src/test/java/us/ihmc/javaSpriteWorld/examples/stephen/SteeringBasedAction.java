package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.*;

public class SteeringBasedAction
{
   private final EnabledBehaviors enabledBehaviors;
   private final SLAMManager slamManager;
   private final FlagManager flagManager;

   private List<Pair<Point2D, Vector2D>> locationOfAllFoodInBodyFrame = new ArrayList<>();
   private List<Pair<Point2D, Vector2D>> locationOfAllPredators = new ArrayList<>();
   private List<Pair<Vector2D, MutableDouble>> rangeSensorData = new ArrayList<>();

   private Pair<Point2D, Integer> filteredSensedFlag;

   private static final double alphaSensedFlag = 0.33;
   private static final double alphaSensedFood = 0.33;
   private static final double alphaPredatorPosition = 0.33;
   private static final double alphaPredatorVelocity = 0.2;
   private static final double alphaRangeSensor = 0.5;

   private final double wallAngularCostRange = Math.toRadians(60.0);
   private final double wallAngularDeadband = Math.toRadians(40.0);
   private final double foodAngularCostRange = Math.toRadians(180.0);
   private final double predatorAngularCostRange = Math.toRadians(80.0);
   private final double goToFlagAngularCostRange = Math.toRadians(180.0);
   private final double avoidFlagAngularCostRange = Math.toRadians(130.0);

   private static final double foodWeightAtMaxHealth = 0.7;
   private static final double foodWeightAtMinHealth = 1.6;

   private double foodWeight = foodWeightAtMaxHealth;
   private final double predatorWeight = 0.2;
   private final double wallWeight = 2.2;
   private final double avoidFlagWeight = 2.5;
   private final double flagWeight = 0.7;
   private final double exploreAreaWeight = 0.4;

   private final double baseWall = 2.4;
   private double baseFood = 2.0;
   private final double basePredator = 1.75;
   private final double baseAvoidFlag = 1.75;

   private double previousHeading = Double.NaN;
   private static final double alphaHeading = 0.3;

   private double minSensedWallHeading = Double.NaN;
   private double minSensedWallDistance = Double.NaN;
   private double minSensedWallPenalty = Double.NaN;
   private final double wallDistanceDeadband = 0.1;

   private final List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();

   public SteeringBasedAction(SLAMManager slamManager, FlagManager flagManager, EnabledBehaviors enabledBehaviors)
   {
      this.enabledBehaviors = enabledBehaviors;
      this.slamManager = slamManager;
      this.flagManager = flagManager;
   }

   public void computeAction(double[] totalAction)
   {
      responseDescriptions.clear();

      // wall
      if (enabledBehaviors.isWallEnabled())
      {
         updateMinWallData();
         responseDescriptions.add(this::computeWallPenaltyAtAngle);
      }

      // food
      if (enabledBehaviors.isFoodEnabled())
      {
         for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
         {
            computeFoodReward(locationOfAllFoodInBodyFrame.get(i).getLeft());
         }
      }

      // predators
      if (enabledBehaviors.isPredatorEnabled())
      {
         for (int i = 0; i < locationOfAllPredators.size(); i++)
         {
            computePredatorPenalty(locationOfAllPredators.get(i).getLeft());
         }
      }

      // chase flag
      if (enabledBehaviors.isFlagEnabled())
      {
         flagManager.update();

         if (flagManager.isInDeliverFlagMode())
         {
            Point2D dropPointWorld = new Point2D(10.0, 10.0);
            Point2D dropPointBody = new Point2D();
            worldFrameToBodyFrame(dropPointWorld, dropPointBody, slamManager.getHeading(), slamManager.getXYPosition());

            computeGoToFlagReward(dropPointBody, flagWeight);

            if (flagManager.getFlagIdToChase() != 5)
            {
               computeAvoidFlagPenalty(filteredSensedFlag.getLeft());
            }
         }
         else
         {
            boolean detectedFlagShouldBeRetrieved = filteredSensedFlag.getRight() == flagManager.getFlagIdToChase();
            if (detectedFlagShouldBeRetrieved)
            {
               computeGoToFlagReward(filteredSensedFlag.getLeft(), flagWeight);
            }
            else
            {
               if (flagManager.hasDetectedNextFlag())
               {
                  Point2D nextFlagLocationInWorld = flagManager.getNextFlagLocation();
                  Point2D nextFlagLocationInBodyFrame = new Point2D();
                  worldFrameToBodyFrame(nextFlagLocationInWorld, nextFlagLocationInBodyFrame, slamManager.getHeading(), slamManager.getXYPosition());
                  computeGoToFlagReward(nextFlagLocationInBodyFrame, flagWeight);
               }
               else
               {
//                  Point2D areaToExploreInWorldFrame = flagManager.getAreaToExplore();
//                  Point2D areaToExploreInBodyFrame = new Point2D();
//                  worldFrameToBodyFrame(areaToExploreInWorldFrame, areaToExploreInBodyFrame, slamManager.getHeading(), slamManager.getXYPosition());
//                  computeGoToFlagReward(areaToExploreInWorldFrame, exploreAreaReward);
               }
            }
         }
      }

      double maxRewardHeading = getMaxRewardHeading(responseDescriptions);

      double filteredHeading = 0.0;
      if (Double.isNaN(previousHeading))
      {
         filteredHeading = maxRewardHeading;
      }
      else
      {
         double angularDifference = EuclidCoreTools.angleDifferenceMinusPiToPi(maxRewardHeading, previousHeading);
         filteredHeading = EuclidCoreTools.trimAngleMinusPiToPi(previousHeading + alphaHeading * angularDifference);
      }

      previousHeading = filteredHeading;

      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 4.0;
      double velocity = slamManager.getFilteredVelocity();

      computeActionGivenHeading(totalAction, maxRewardHeading, velocityWhenAligned, kAcceleration, kTurn, velocity);
   }

   public static void computeActionGivenHeading(double[] totalAction,
                                                double maxRewardHeading,
                                                double velocityWhenAligned,
                                                double kAcceleration,
                                                double kTurn,
                                                double velocity)
   {
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);
      double deltaDesiredHeading = maxRewardHeading;

      double acceleration;

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = EuclidCoreTools.interpolate(velocityWhenAligned, 0.0, Math.abs(deltaDesiredHeading / angleToStopAndTurn));

         // don't break if roughly aligned
         acceleration = Math.max(kAcceleration * (targetVelocity - velocity), 0.0);
      }
      else
      {
         targetVelocity = 0.0;

         // break if necessary when not aligned
         acceleration = kAcceleration * (targetVelocity - velocity);
      }

      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = acceleration;
      totalAction[1] = turningAction;
   }

   public static double getMaxRewardHeading(List<ObjectResponseDescription> responseDescriptions)
   {
      double angle = -Math.PI;
      double maxReward = Double.NEGATIVE_INFINITY;
      double maxRewardHeading = Double.NaN;

      while (angle <= Math.PI)
      {
         double reward = 0.0;
         for (int i = 0; i < responseDescriptions.size(); i++)
         {
            reward += responseDescriptions.get(i).getRewardAtAngle(angle);
         }

         if (reward > maxReward)
         {
            maxReward = reward;
            maxRewardHeading = angle;
         }

         angle += 0.01;
      }
      return maxRewardHeading;
   }

   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      if (rangeSensorData.isEmpty())
      {
         for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
         {
            Pair<Vector2D, Double> rangeData = vectorsAndDistancesToWallInBodyFrame.get(i);
            rangeSensorData.add(Pair.of(new Vector2D(rangeData.getLeft()), new MutableDouble(rangeData.getRight())));
         }
      }
      else
      {
         for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
         {
            double sensedRange = vectorsAndDistancesToWallInBodyFrame.get(i).getRight();
            MutableDouble filteredRange = rangeSensorData.get(i).getRight();
            filteredRange.setValue(filter(alphaRangeSensor, sensedRange, filteredRange.getValue()));
         }
      }
   }

   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      if (locationOfAllFoodInBodyFrame.isEmpty())
      {
         for (int i = 0; i < locationOfAllFood.size(); i++)
         {
            Triple<Integer, Point2D, Vector2D> foodData = locationOfAllFood.get(i);
            locationOfAllFoodInBodyFrame.add(Pair.of(new Point2D(foodData.getMiddle()), new Vector2D(foodData.getRight())));
         }
      }
      else
      {
         for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
         {
            Point2D positionToSetFiltered = locationOfAllFoodInBodyFrame.get(i).getLeft();
            Triple<Integer, Point2D, Vector2D> newData = locationOfAllFood.get(i);

            positionToSetFiltered.setX(filter(alphaSensedFood, newData.getMiddle().getX(), positionToSetFiltered.getX()));
            positionToSetFiltered.setY(filter(alphaSensedFood, newData.getMiddle().getY(), positionToSetFiltered.getY()));
         }
      }
   }

   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      if (this.locationOfAllPredators.isEmpty())
      {
         for (int i = 0; i < locationOfAllPredators.size(); i++)
         {
            Pair<Point2D, Vector2D> predatorData = locationOfAllPredators.get(i);
            this.locationOfAllPredators.add(Pair.of(new Point2D(predatorData.getLeft()), new Vector2D(predatorData.getRight())));
         }
      }
      else
      {
         for (int i = 0; i < this.locationOfAllPredators.size(); i++)
         {
            Pair<Point2D, Vector2D> dataToSet = this.locationOfAllPredators.get(i);
            Pair<Point2D, Vector2D> newData = locationOfAllPredators.get(i);

            Point2D predatorPosition = dataToSet.getLeft();

            Vector2D predatorVelocity = dataToSet.getRight();
            predatorVelocity.interpolate(newData.getRight(), alphaPredatorVelocity);

            predatorPosition.addX(predatorVelocity.getX() * SLAMManager.dt * 0.5);
            predatorPosition.addY(predatorVelocity.getY() * SLAMManager.dt * 0.5);

            predatorPosition.interpolate(newData.getLeft(), alphaPredatorPosition);
            predatorVelocity.set(newData.getRight());
         }
      }
   }

   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      if (this.filteredSensedFlag == null || this.filteredSensedFlag.getRight() != positionInBodyFrameAndIdOfClosestFlag.getRight())
      {
         this.filteredSensedFlag = Pair.of(new Point2D(positionInBodyFrameAndIdOfClosestFlag.getLeft()), positionInBodyFrameAndIdOfClosestFlag.getRight());
      }
      else
      {
         Point2D previousPosition = this.filteredSensedFlag.getLeft();
         Point2D newPosition = positionInBodyFrameAndIdOfClosestFlag.getLeft();
         Point2D filteredPosition = filter(alphaSensedFlag, newPosition, previousPosition);
         this.filteredSensedFlag = Pair.of(filteredPosition, positionInBodyFrameAndIdOfClosestFlag.getRight());
      }
   }

   public void senseHealth(double health)
   {
      foodWeight = EuclidCoreTools.interpolate(foodWeightAtMaxHealth, foodWeightAtMinHealth, health / 100.0);
   }

   private void computeFoodReward(Tuple2DReadOnly foodInBodyFrame)
   {
      double deadband = 0.15;
      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
      double heading = distance < deadband ? 0.0 : headingFromVector(foodInBodyFrame);

      double reward = foodWeight * Math.pow(baseFood, - distance);
      responseDescriptions.add(new RampedAngularReward(heading, foodAngularCostRange, reward));
   }

   private void computePredatorPenalty(Tuple2DReadOnly predatorInBodyFrame)
   {
      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      double reward = -predatorWeight * Math.pow(basePredator, - distance);
      double heading = headingFromVector(predatorInBodyFrame);
      responseDescriptions.add(new RampedAngularReward(heading, predatorAngularCostRange, reward));
   }

   private void computeAvoidFlagPenalty(Tuple2DReadOnly flagInBodyFrame)
   {
      double heading = headingFromVector(flagInBodyFrame);
      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      double cost = -avoidFlagWeight * Math.pow(baseAvoidFlag, - distance);
      responseDescriptions.add(new RampedAngularReward(heading, avoidFlagAngularCostRange, cost));
   }

   private void computeGoToFlagReward(Tuple2DReadOnly flagInBodyFrame, double reward)
   {
      double deadband = 0.5;
      if (EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY()) < deadband)
      {
         responseDescriptions.add(new RampedAngularReward(0.0, goToFlagAngularCostRange, reward));
      }
      else
      {
         responseDescriptions.add(new RampedAngularReward(headingFromVector(flagInBodyFrame), goToFlagAngularCostRange, reward));
      }
   }

   private void updateMinWallData()
   {
      int minWallIndex = 0;
      minSensedWallDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < rangeSensorData.size(); i++)
      {
         Pair<Vector2D, MutableDouble> vectorAndDistanceToWall = rangeSensorData.get(i);
         if (vectorAndDistanceToWall.getRight().getValue() < minSensedWallDistance)
         {
            minSensedWallDistance = vectorAndDistanceToWall.getRight().getValue();
            minWallIndex = i;
         }
      }

      minSensedWallPenalty = -wallWeight * Math.pow(baseWall, - Math.max(minSensedWallDistance - wallDistanceDeadband, 0.0));
      minSensedWallHeading = headingFromVector(rangeSensorData.get(minWallIndex).getLeft());
   }

   private double computeWallPenaltyAtAngle(double headingInBodyFrame)
   {
      double penaltyFromNearestSensors = computeWallPenaltyFromNearestSensors(headingInBodyFrame);

      double penaltyFromMinWallDistance = Double.POSITIVE_INFINITY;
      double angularDifference = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingInBodyFrame, minSensedWallHeading));
      angularDifference = Math.max(0.0, angularDifference - wallAngularDeadband);
      if (angularDifference < wallAngularCostRange)
      {
         double multiplier = (1.0 - angularDifference / wallAngularCostRange);
         penaltyFromMinWallDistance = minSensedWallPenalty * multiplier;
      }

      double wallPenaltyAtAngle = Math.min(penaltyFromNearestSensors, penaltyFromMinWallDistance);
      return wallPenaltyAtAngle;
   }

   private double computeWallPenaltyFromNearestSensors(double headingInBodyFrame)
   {
      double maxAngleFromPerpendicular = Math.toRadians(30.0);
      if (Math.abs(headingInBodyFrame) > maxAngleFromPerpendicular + 0.5 * Math.PI)
      {
         return 0.0;
      }

      Pair<Integer, Integer> nearestSensorIndices = getNearestSensorIndices(headingInBodyFrame);

      int i1 = nearestSensorIndices.getLeft();
      int i2 = nearestSensorIndices.getLeft();

      double dAng1 = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingInBodyFrame, headingFromVector(rangeSensorData.get(i1).getLeft())));
      double dAng2 = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingInBodyFrame, headingFromVector(rangeSensorData.get(i2).getLeft())));

      double dist1 = rangeSensorData.get(i1).getRight().getValue();
      double dist2 = rangeSensorData.get(i2).getRight().getValue();

      double dist = (dist1 * dAng2 + dist2 * dAng1) / (dAng1 + dAng2);
      return - wallWeight * Math.pow(baseWall, - Math.max(dist - wallDistanceDeadband, 0.0));
   }

   private Pair<Integer, Integer> getNearestSensorIndices(double heading)
   {
      double angle1Min = Double.POSITIVE_INFINITY;
      int index1Min = -1;
      int index2Min = -1;

      for (int i = 0; i < rangeSensorData.size(); i++)
      {
         double rangeHeading = headingFromVector(rangeSensorData.get(i).getLeft());
         double angleDiff = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(rangeHeading, heading));
         if (angleDiff < angle1Min)
         {
            index2Min = index1Min;
            index1Min = i;
            angle1Min = angleDiff;
         }
      }

      return Pair.of(index1Min, index2Min);
   }

   public void reset()
   {
      locationOfAllFoodInBodyFrame.clear();
      locationOfAllPredators.clear();
      filteredSensedFlag = null;
      previousHeading = Double.NaN;
      foodWeight = foodWeightAtMaxHealth;
   }
}
