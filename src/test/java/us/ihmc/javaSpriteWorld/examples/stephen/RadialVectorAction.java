package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

import java.util.ArrayList;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.*;

public class RadialVectorAction
{
   private final SLAMManager slamManager;
   private final FlagManager flagManager;

   // environment state
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag;
   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;

   // behavior parameters
   private final Vector2D wallForce = new Vector2D();
   private final Vector2D foodForce = new Vector2D();
   private final Vector2D predatorForce = new Vector2D();
   private final Vector2D flagForce = new Vector2D();

   private final Vector2D totalForce = new Vector2D();

   private final double maxForceWall = 0.4;
   private final double maxForceFood = 1.25;
   private final double maxForcePredator = 1.2;
   private final double maxForceAvoidFlagWhileDelivering = 3.0;
   private final double flagAttractionForceMagnitude = 0.5;
   private final double exploreAreaForceMagnitude = 0.4;

   private final double minAngleToPenalizeFood = Math.toRadians(60.0);
   private final double extraFoodDistanceIfBehind = 1.0;
   private final double proximityNearWallToIgnoreFood = 0.8;

   private final double baseWall = 2.75;
   private double baseFood = 2.2;
   private final double basePredator = 2.0;
   private final double baseFlag = 2.5;

   // toggle sub-behaviors
   private boolean enableWall = true;
   private boolean enableFood = true;
   private boolean enablePredators = true;
   private boolean enableFlags = false;

   public RadialVectorAction(SLAMManager slamManager, FlagManager flagManager)
   {
      this.slamManager = slamManager;
      this.flagManager = flagManager;
   }

   public void computeAction(double[] totalAction)
   {
      wallForce.setToZero();
      foodForce.setToZero();
      predatorForce.setToZero();
      flagForce.setToZero();

      // wall
      computeWallForce();

      // food
      for (int i = 0; i < locationOfAllFoodInBodyFrame.size(); i++)
      {
         computeFoodForce(foodForce, locationOfAllFoodInBodyFrame.get(i).getMiddle());
      }

      // predators
      for (int i = 0; i < locationOfAllPredators.size(); i++)
      {
         computePredatorForce(predatorForce, locationOfAllPredators.get(i).getLeft());
      }

      // chase flag
      if (flagManager.isInDeliverFlagMode())
      {
         Point2D dropPointWorld = new Point2D(9.0, 9.0);
         Point2D dropPointBody = new Point2D();
         worldFrameToBodyFrame(dropPointWorld, dropPointBody, slamManager.getHeading(), slamManager.getXYPosition());

         computeGoToFlagForce(flagForce, dropPointBody);
         computeAvoidFlagForce(flagForce, positionInBodyFrameAndIdOfClosestFlag.getLeft());
      }
      else
      {
         boolean detectedFlagShouldBeRetrieved = positionInBodyFrameAndIdOfClosestFlag.getRight() == flagManager.getFlagIdToChase();
         if (detectedFlagShouldBeRetrieved)
         {
            computeGoToFlagForce(flagForce, positionInBodyFrameAndIdOfClosestFlag.getLeft());
         }
         else
         {
            if (flagManager.hasDetectedNextFlag())
            {
               computeGoToWorldCoordinate(flagForce, flagManager.getNextFlagLocation(), flagAttractionForceMagnitude);
            }
            else
            {
               computeGoToWorldCoordinate(flagForce, flagManager.getAreaToExplore(), exploreAreaForceMagnitude);
            }
         }
      }

      toggleOffSubBehaviors();
      totalForce.setX(wallForce.getX() + predatorForce.getX() + foodForce.getX() + flagForce.getX());
      totalForce.setY(wallForce.getY() + predatorForce.getY() + foodForce.getY() + flagForce.getY());

      double totalForceMagnitude = totalForce.length();
      double percentageWall = wallForce.length() / totalForceMagnitude;
      double percentageFood = foodForce.length() / totalForceMagnitude;
      double percentagePredator = predatorForce.length() / totalForceMagnitude;
      double percentageFlagAttractor = flagForce.length() / totalForceMagnitude;
      double percentageFlagRepulsive = flagForce.length() / totalForceMagnitude;

      // convert to acceleration and steering action
      double deltaDesiredHeading = headingFromVector(totalForce.getX(), totalForce.getY());

      double velocityWhenAligned = 3.0;
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = velocityWhenAligned - (velocityWhenAligned / angleToStopAndTurn) * Math.abs(deltaDesiredHeading);
      }
      else
      {
         targetVelocity = 0.0;
      }

      double kAcceleration = 1.5;
      double accelerationAction = kAcceleration * (targetVelocity - slamManager.getVelocity());

      double kTurn = 2.0;
      double turningAction = kTurn * deltaDesiredHeading;

      totalAction[0] = accelerationAction;
      totalAction[1] = turningAction;
   }

   private void computeWallForce()
   {
      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensorData = vectorsAndDistancesToWallInBodyFrame.get(i);
         Vector2D sensedWallForce = new Vector2D(sensorData.getLeft());
         sensedWallForce.normalize();
         sensedWallForce.scale(-1.0);
         sensedWallForce.scale(maxForceWall * Math.pow(baseWall, - sensorData.getRight()));

         wallForce.add(sensedWallForce);
      }
   }

   private void computeFoodForce(Vector2D forceToAddTo, Point2D foodInBodyFrame)
   {
      Vector2D force = new Vector2D();

      // Dead reckoning seems off...
      double distanceToWall = computeDistanceToWall(new Point2D(foodInBodyFrame));
      if (distanceToWall < proximityNearWallToIgnoreFood)
      {
         return;
      }

      force.set(foodInBodyFrame);
      force.normalize();

      double distance = EuclidCoreTools.norm(foodInBodyFrame.getX(), foodInBodyFrame.getY());
      double angleFromStraightAhead = headingFromVector(foodInBodyFrame);
      if (Math.abs(angleFromStraightAhead) > minAngleToPenalizeFood)
      {
         double angleFromStraightBehind = EuclidCoreTools.angleDifferenceMinusPiToPi(angleFromStraightAhead, Math.PI);
         distance += extraFoodDistanceIfBehind * (1.0 - Math.abs(angleFromStraightBehind) / (Math.PI - minAngleToPenalizeFood));
      }

      force.scale(maxForceFood * Math.pow(baseFood, - distance));
      forceToAddTo.add(force);
   }

   private void computePredatorForce(Vector2D forceToAddTo, Tuple2DReadOnly predatorInBodyFrame)
   {
      Vector2D force = new Vector2D();

      force.set(predatorInBodyFrame);
      force.negate();
      force.normalize();

      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      force.scale(maxForcePredator * Math.pow(basePredator, - distance));
      forceToAddTo.add(force);
   }

   private void computeAvoidFlagForce(Vector2D forceToAddTo, Tuple2DReadOnly flagInBodyFrame)
   {
      Vector2D force = new Vector2D();

      double fieldOfViewForAvoiding = Math.toRadians(45.0);
      double angle = headingFromVector(flagInBodyFrame);
      if (Math.abs(angle) > fieldOfViewForAvoiding)
      {
         return;
      }

      force.set(flagInBodyFrame);
      force.negate();
      force.normalize();

      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      force.scale(maxForceAvoidFlagWhileDelivering * Math.pow(baseFlag, - distance));
      forceToAddTo.add(force);
   }

   private void computeGoToFlagForce(Vector2D forceToAddTo, Tuple2DReadOnly flagInBodyFrame)
   {
      Vector2D force = new Vector2D();

      force.set(flagInBodyFrame);
      force.normalize();

      force.scale(flagAttractionForceMagnitude);
      forceToAddTo.add(force);
   }

   private void computeGoToWorldCoordinate(Vector2D forceToAddTo, Point2DReadOnly worldCoordinate, double forceMagnitude)
   {
      Point2D bodyFrameCoordinate = new Point2D();
      worldFrameToBodyFrame(worldCoordinate, bodyFrameCoordinate, slamManager.getHeading(), slamManager.getXYPosition());

      Vector2D force = new Vector2D(bodyFrameCoordinate);
      force.normalize();
      force.scale(forceMagnitude);

      forceToAddTo.add(force);
   }

   private double computeDistanceToWall(Point2DReadOnly foodInBodyFrame)
   {
      Point2D foodInWorld = new Point2D();
      bodyFrameToWorldFrame(foodInBodyFrame, foodInWorld, slamManager.getHeading(), slamManager.getXYPosition());
      return Math.min(Math.min(foodInWorld.getX(), foodInWorld.getY()), Math.min(10.0 - foodInWorld.getX(), 10.0 - foodInWorld.getY()));
   }

   private void toggleOffSubBehaviors()
   {
      if (!enableWall)
      {
         wallForce.setToZero();
      }
      if (!enableFood)
      {
         foodForce.setToZero();
      }
      if (!enablePredators)
      {
         predatorForce.setToZero();
      }
      if (!enableFlags)
      {
         flagForce.setToZero();
      }
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

}
