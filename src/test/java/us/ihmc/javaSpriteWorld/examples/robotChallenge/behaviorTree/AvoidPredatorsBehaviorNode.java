package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;

public class AvoidPredatorsBehaviorNode  implements BehaviorTreeAction
{
//   public static final double
   
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

   private void computePredatorPenalty(Tuple2DReadOnly predatorInBodyFrame)
   {
      double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
      double reward = - Math.pow(basePredator, - distance);
      double heading = headingFromVector(predatorInBodyFrame);
      responseDescriptions.add(new RampedAngularReward(heading, predatorAngularCostRange, reward));
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      responseDescriptions.clear();
      for (int i = 0; i < sensors.getLocationOfAllPredators().size(); i++)
      {
         computePredatorPenalty(sensors.getLocationOfAllPredators().get(i).getLeft());
      }

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
