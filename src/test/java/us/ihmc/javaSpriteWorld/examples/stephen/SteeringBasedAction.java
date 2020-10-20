package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tools.AxisAngleTools;
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
   private Pair<Point2D, Integer> filteredSensedFlag;
   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;

   // Higher weight rewards/penalties for objects in front of the robot
   private static final double rewardScaleWhenBehindRobot = 0.4;
   
   private static final double alphaSensedFlag = 0.3;
   private static final double alphaSensedFood = 0.3;
   private static final double alphaSensedPredators = 0.1;

   private final double wallAngularCostRange = Math.toRadians(40.0);
   private final double foodAngularCostRange = Math.toRadians(180.0);
   private final double predatorAngularCostRange = Math.toRadians(80.0);
   private final double goToFlagAngularCostRange = Math.toRadians(180.0);
   private final double avoidFlagAngularCostRange = Math.toRadians(130.0);

   private final double foodWeight = 1.0;
   private final double predatorWeight = 0.1;
   private final double wallWeight = 2.2;
   private final double avoidFlagWeight = 3.0;
   private final double flagWeight = 0.7;
   private final double exploreAreaWeight = 0.4;

   private final double baseWall = 2.4;
   private double baseFood = 2.0;
   private final double basePredator = 1.75;
   private final double baseAvoidFlag = 1.75;

   private double previousHeading = Double.NaN;
   private static final double alphaHeading = 0.7;

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
         computeWallAction();
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
            Point2D dropPointWorld = new Point2D(9.0, 9.0);
            Point2D dropPointBody = new Point2D();
            worldFrameToBodyFrame(dropPointWorld, dropPointBody, slamManager.getHeading(), slamManager.getXYPosition());

            computeGoToFlagReward(dropPointBody, flagWeight);
            computeAvoidFlagPenalty(filteredSensedFlag.getLeft());
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
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);
      double deltaDesiredHeading = maxRewardHeading;

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = EuclidCoreTools.interpolate(velocityWhenAligned, 0.0, Math.abs(deltaDesiredHeading / angleToStopAndTurn));
      }
      else
      {
         targetVelocity = 0.0;
      }

      double kAcceleration = 3.0;
      double accelerationAction = kAcceleration * (targetVelocity - slamManager.getFilteredVelocity());

      double kTurn = 4.0;
      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = accelerationAction;
      totalAction[1] = turningAction;
   }

   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      this.vectorsAndDistancesToWallInBodyFrame = vectorsAndDistancesToWallInBodyFrame;
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
            predatorPosition.addX(SLAMManager.dt * predatorVelocity.getX());
            predatorPosition.addY(SLAMManager.dt * predatorVelocity.getY());
            predatorPosition.interpolate(newData.getLeft(), alphaSensedPredators);
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

   private void computeWallAction()
   {
      int minWallIndex = 0;
      double minWallDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> vectorAndDistanceToWall = vectorsAndDistancesToWallInBodyFrame.get(i);
         if (vectorAndDistanceToWall.getRight() < minWallDistance)
         {
            minWallDistance = vectorAndDistanceToWall.getRight();
            minWallIndex = i;
         }
      }

      double penalty = -wallWeight * Math.pow(baseWall, - minWallDistance);
      Vector2D sensorVector = vectorsAndDistancesToWallInBodyFrame.get(minWallIndex).getLeft();
      responseDescriptions.add(new ObjectResponseDescription(headingFromVector(sensorVector), wallAngularCostRange, penalty));
   }

   private void computeFoodReward(Tuple2DReadOnly foodInBodyFrame)
   {
      double deadband = 0.15;
      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
      double heading = distance < deadband ? 0.0 : headingFromVector(foodInBodyFrame);

      double reward = foodWeight * Math.pow(baseFood, - distance);
      responseDescriptions.add(new ObjectResponseDescription(heading, foodAngularCostRange, reward));
   }

   private void computePredatorPenalty(Tuple2DReadOnly predatorInBodyFrame)
   {
      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      double reward = -predatorWeight * Math.pow(basePredator, - distance);
      double heading = headingFromVector(predatorInBodyFrame);
      responseDescriptions.add(new ObjectResponseDescription(heading, predatorAngularCostRange, reward));
   }

   private void computeAvoidFlagPenalty(Tuple2DReadOnly flagInBodyFrame)
   {
      double heading = headingFromVector(flagInBodyFrame);
      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      double cost = -avoidFlagWeight * Math.pow(baseAvoidFlag, - distance);
      responseDescriptions.add(new ObjectResponseDescription(heading, avoidFlagAngularCostRange, cost));
   }

   private void computeGoToFlagReward(Tuple2DReadOnly flagInBodyFrame, double reward)
   {
      double deadband = 0.5;
      if (EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY()) < deadband)
      {
         responseDescriptions.add(new ObjectResponseDescription(0.0, goToFlagAngularCostRange, reward));
      }
      else
      {
         responseDescriptions.add(new ObjectResponseDescription(headingFromVector(flagInBodyFrame), goToFlagAngularCostRange, reward));
      }
   }

   private static class ObjectResponseDescription
   {
      private final double headingOfObjectInBodyFrame;
      private final double angularRange;
      private final double rewardWhenFacingObject;

      public ObjectResponseDescription(double headingOfObjectInBodyFrame, double angularRange, double rewardWhenFacingObject)
      {
         this.headingOfObjectInBodyFrame = headingOfObjectInBodyFrame;
         this.angularRange = angularRange;
         this.rewardWhenFacingObject = rewardWhenFacingObject;
      }

      public double getRewardAtAngle(double headingInBodyFrame)
      {
         double angularDifference = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingInBodyFrame, headingOfObjectInBodyFrame));
         if (angularDifference > angularRange)
         {
            return 0.0;
         }
         else
         {
            double multiplier = EuclidCoreTools.interpolate(1.0, rewardScaleWhenBehindRobot, Math.abs(headingOfObjectInBodyFrame) / Math.PI);
            return multiplier * rewardWhenFacingObject * (1.0 - angularDifference / angularRange);
         }
      }
   }

}
