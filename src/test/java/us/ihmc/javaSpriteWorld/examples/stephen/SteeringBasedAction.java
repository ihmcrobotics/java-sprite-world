package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;
import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.worldFrameToBodyFrame;

public class SteeringBasedAction
{
   private final SLAMManager slamManager;
   private final FlagManager flagManager;

   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag;
   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;

   private final double wallAngularCostRange = Math.toRadians(75.0);
   private final double foodAngularCostRange = Math.toRadians(180.0);
   private final double predatorAngularCostRange = Math.toRadians(80.0);
   private final double goToFlagAngularCostRange = Math.toRadians(180.0);
   private final double avoidFlagAngularCostRange = Math.toRadians(45.0);

   private final double maxForceWall = 2.5;
   private final double maxForceFood = 1.0;
   private final double maxForcePredator = 1.2;
   private final double maxForceAvoidFlagWhileDelivering = 3.0;
   private final double flagAttractionForceMagnitude = 0.5;
   private final double exploreAreaForceMagnitude = 0.4;

   private final double minAngleToPenalizeFood = Math.toRadians(60.0);
   private final double extraFoodDistanceIfBehind = 1.0;
   private final double proximityNearWallToIgnoreFood = 0.8;

   private final double baseWall = 2.65;
   private double baseFood = 2.2;
   private final double basePredator = 2.0;
   private final double baseFlag = 2.5;

   private final List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();

   public SteeringBasedAction(SLAMManager slamManager, FlagManager flagManager)
   {
      this.slamManager = slamManager;
      this.flagManager = flagManager;
   }

   public void computeAction(double[] totalAction)
   {
      responseDescriptions.clear();

      // wall
      computeWallAction();

      // food
      for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
      {
         computeFoodAction(locationOfAllFoodInBodyFrame.get(i).getMiddle());
      }

      // predators
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         computePredatorForce(locationOfAllPredators.get(i).getLeft());
      }

      // chase flag
      if (flagManager.isInDeliverFlagMode())
      {
         Vector2D dropPointWorld = new Vector2D(9.0, 9.0);
         Vector2D dropPointBody = new Vector2D();
         worldFrameToBodyFrame(dropPointWorld, dropPointBody, slamManager.getHeading());

         computeGoToFlagForce(dropPointBody);
         computeAvoidFlagForce(positionInBodyFrameAndIdOfClosestFlag.getLeft());
      }
      else
      {
         boolean detectedFlagShouldBeRetrieved = positionInBodyFrameAndIdOfClosestFlag.getRight() == flagManager.getFlagIdToChase();
         if (detectedFlagShouldBeRetrieved)
         {
            computeGoToFlagForce(positionInBodyFrameAndIdOfClosestFlag.getLeft());
         }
         else
         {
            // TODO
         }
      }

      // find max cost (rename later...) heading
      double angle = -Math.PI;
      double maxCost = Double.NEGATIVE_INFINITY;
      double maxCostHeading = Double.NaN;

      while (angle <= Math.PI)
      {
         double cost = 0.0;
         for (int i = 0; i < responseDescriptions.size(); i++)
         {
            cost += responseDescriptions.get(i).getCostAtAngle(angle);
         }

         if (cost > maxCost)
         {
            maxCost = cost;
            maxCostHeading = angle;
         }

         angle += 0.01;
      }

      double velocityWhenAligned = 3.0;
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);
      double deltaDesiredHeading = maxCostHeading;

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = velocityWhenAligned - (velocityWhenAligned / angleToStopAndTurn) * Math.abs(deltaDesiredHeading);
      }
      else
      {
         targetVelocity = 0.0;
      }

      double kAcceleration = 3.0;
      double accelerationAction = kAcceleration * (targetVelocity - slamManager.getVelocity());

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
      this.locationOfAllFoodInBodyFrame = locationOfAllFood;
   }

   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      this.positionInBodyFrameAndIdOfClosestFlag = positionInBodyFrameAndIdOfClosestFlag;
   }

   private void computeWallAction()
   {
//      double cost = -maxForceWall * Math.pow(baseWall, -wallDistance);
//      responseDescriptions.add(new ObjectResponseDescription(0.0, wallAngularCostRange, cost));
   }

   private void computeFoodAction(Tuple2DReadOnly foodInBodyFrame)
   {
//      double distanceToWall = computeDistanceToWall(foodInBodyFrame);
//      if (distanceToWall < proximityNearWallToIgnoreFood)
//      {
//         return;
//      }
//
//      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
//      double angleFromStraightAhead = headingFromVector(foodInBodyFrame);
//      if (Math.abs(angleFromStraightAhead) > minAngleToPenalizeFood)
//      {
//         double angleFromStraightBehind = EuclidCoreTools.angleDifferenceMinusPiToPi(angleFromStraightAhead, Math.PI);
//         distance += extraFoodDistanceIfBehind * (1.0 - Math.abs(angleFromStraightBehind) / (Math.PI - minAngleToPenalizeFood));
//      }
//
//      double cost = maxForceFood * Math.pow(baseFood, - distance);
//      double heading = headingFromVector(foodInBodyFrame);
//      responseDescriptions.add(new ObjectResponseDescription(heading, foodAngularCostRange, cost));
   }

   private void computePredatorForce(Tuple2DReadOnly predatorInBodyFrame)
   {
      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      double cost = - maxForcePredator * Math.pow(basePredator, - distance);
      double heading = headingFromVector(predatorInBodyFrame);
      responseDescriptions.add(new ObjectResponseDescription(heading, predatorAngularCostRange, cost));
   }

   private void computeAvoidFlagForce(Tuple2DReadOnly flagInBodyFrame)
   {
      double heading = headingFromVector(flagInBodyFrame);
      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      double cost = - maxForceAvoidFlagWhileDelivering * Math.pow(baseFlag, - distance);
      responseDescriptions.add(new ObjectResponseDescription(heading, avoidFlagAngularCostRange, cost));
   }

   private void computeGoToFlagForce(Tuple2DReadOnly flagInBodyFrame)
   {
      responseDescriptions.add(new ObjectResponseDescription(headingFromVector(flagInBodyFrame), goToFlagAngularCostRange, flagAttractionForceMagnitude));
   }

   private class ObjectResponseDescription
   {
      private final double heading;
      private final double angularRange;
      private final double cost;

      public ObjectResponseDescription(double heading, double angularRange, double cost)
      {
         this.heading = heading;
         this.angularRange = angularRange;
         this.cost = cost;
      }

      public double getHeading()
      {
         return heading;
      }

      public double getAngularRange()
      {
         return angularRange;
      }

      public double getCost()
      {
         return cost;
      }

      public double getCostAtAngle(double angle)
      {
         double angularDifference = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(angle, heading));
         if (angularDifference > angularRange)
         {
            return 0.0;
         }
         else
         {
            return cost * (1.0 - angularDifference / angularRange);
         }
      }
   }

}
