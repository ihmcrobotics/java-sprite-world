package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class AvoidObstaclesNode implements BehaviorTreeAction
{
   private final AvoidWallsBehaviorNode avoidWallsNode;
   private final AvoidPredatorsBehaviorNode avoidPredatorsNode;
   private RobotBehaviorSensors sensors;
   private RobotBehaviorActuators actuators;
   private RobotBehaviorEnvironment environment;

   public AvoidObstaclesNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators, RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.environment = environment;

      avoidWallsNode = new AvoidWallsBehaviorNode(sensors, actuators, environment);
      avoidPredatorsNode = new AvoidPredatorsBehaviorNode(sensors, actuators);
   }

   @Override
   public double evaluateUtility()
   {
      return Math.max(avoidWallsNode.evaluateUtility(), avoidPredatorsNode.evaluateUtility());
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (avoidWallsNode.evaluateUtility() > avoidPredatorsNode.evaluateUtility())
      {
         return avoidWallsNode.tick();
      }
      else
      {
         return avoidPredatorsNode.tick();
      }
   }
}
