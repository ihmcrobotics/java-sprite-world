package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.stephen.ObjectResponseDescription;
import us.ihmc.javaSpriteWorld.examples.stephen.RampedAngularReward;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;

public class AvoidPredatorsBehaviorNode  implements BehaviorTreeAction
{
   public static final double PREDATOR_PROXIMITY_TO_ACTIVATE = 2.0;
   
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;

   private final List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();
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

      double maxRewardHeading = SteeringBasedAction.getMaxRewardHeading(responseDescriptions);

      double[] accelerationAndTurnRate = new double[2];
      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 4.0;

      SteeringBasedAction.computeActionGivenHeading(accelerationAndTurnRate, maxRewardHeading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());

      actuators.setAcceleration(accelerationAndTurnRate[0]);
      actuators.setTurnRate(accelerationAndTurnRate[1]);

      return BehaviorTreeNodeStatus.RUNNING;
   }
}
