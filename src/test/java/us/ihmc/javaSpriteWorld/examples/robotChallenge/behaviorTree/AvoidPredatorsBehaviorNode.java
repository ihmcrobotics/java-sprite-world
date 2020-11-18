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
   private final double predatorAngularCostRange = Math.toRadians(120.0);

   private final double predWeight = 1.0;
   private final double currentHeadingWeight = 0.1;

   private final double[] predatorAction = new double[2];

   public AvoidPredatorsBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public double evaluateUtility()
   {
      responseDescriptions.clear();

      for (int i = 0; i < sensors.getLocationOfAllPredators().size(); i++)
      {
         Point2D predatorInBodyFrame = sensors.getLocationOfAllPredators().get(i).getLeft();
         double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());
         double reward = - predWeight * Math.pow(basePredator, - distance);
         double heading = headingFromVector(predatorInBodyFrame);

         responseDescriptions.add(new RampedAngularReward(heading, predatorAngularCostRange, reward));
      }

      responseDescriptions.add(new RampedAngularReward(0.0, Math.PI, currentHeadingWeight));

      boolean enable = sensors.getClosestPredatorDistance() < PREDATOR_PROXIMITY_TO_ACTIVATE;
      double maxRewardHeading = SteeringBasedAction.getMaxRewardHeading(responseDescriptions);

      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 4.0;

      double utility = enable ? 1.0 : 0.0;
      SteeringBasedAction.computeActionGivenHeading(predatorAction, maxRewardHeading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());
      return utility;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      actuators.setAcceleration(predatorAction[0]);
      actuators.setTurnRate(predatorAction[1]);

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
