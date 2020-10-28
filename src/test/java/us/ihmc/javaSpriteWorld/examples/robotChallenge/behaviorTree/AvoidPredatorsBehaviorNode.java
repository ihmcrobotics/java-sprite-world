package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;

public class AvoidPredatorsBehaviorNode  implements BehaviorTreeAction
{
   public static final double PREDATOR_PROXIMITY_TO_ACTIVATE = 2.0;
   
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;

   private static final double rewardScaleWhenBehindRobot = 0.7;
   private final List<RampedAngularReward> responseDescriptions = new ArrayList<>();
   private final double basePredator = 1.75;
   private final double predatorAngularCostRange = Math.toRadians(80.0);

   public AvoidPredatorsBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      responseDescriptions.clear();
      double closestPredatorDistance = Double.MAX_VALUE;

      for (int i = 0; i < sensors.getLocationOfAllPredators().size(); i++)
      {
         Point2D predatorInBodyFrame = sensors.getLocationOfAllPredators().get(i).getLeft();
         double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
         double reward = - Math.pow(basePredator, - distance);
         double heading = headingFromVector(predatorInBodyFrame);

         if (distance < closestPredatorDistance)
         {
            closestPredatorDistance = distance;
         }

         responseDescriptions.add(new RampedAngularReward(heading, predatorAngularCostRange, reward));
      }

      if (closestPredatorDistance > PREDATOR_PROXIMITY_TO_ACTIVATE)
      {
         return BehaviorTreeNodeStatus.SUCCESS;
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

      double velocityWhenAligned = 3.0;
      double targetVelocity;
      double angleToStopAndTurn = Math.toRadians(60.0);
      double deltaDesiredHeading = maxRewardHeading;

      double kAcceleration = 3.0;
      double kTurn = 4.0;
      double acceleration;

      if (Math.abs(deltaDesiredHeading) < angleToStopAndTurn)
      {
         targetVelocity = EuclidCoreTools.interpolate(velocityWhenAligned, 0.0, Math.abs(deltaDesiredHeading / angleToStopAndTurn));
         acceleration = Math.max(kAcceleration * (targetVelocity - sensors.getVelocity()), 0.0);
      }
      else
      {
         targetVelocity = 0.0;
         acceleration = kAcceleration * (targetVelocity - sensors.getVelocity());
      }

      actuators.setAcceleration(acceleration);
      actuators.setTurnRate(kTurn * deltaDesiredHeading);

      return BehaviorTreeNodeStatus.RUNNING;
   }

   private static class RampedAngularReward
   {
      private final double headingOfObjectInBodyFrame;
      private final double angularRange;
      private final double rewardWhenFacingObject;

      public RampedAngularReward(double headingOfObjectInBodyFrame, double angularRange, double rewardWhenFacingObject)
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
