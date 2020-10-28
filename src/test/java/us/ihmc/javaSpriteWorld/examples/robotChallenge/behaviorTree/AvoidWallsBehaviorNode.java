package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class AvoidWallsBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private RobotBehaviorEnvironment environment;

   public AvoidWallsBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators, RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.environment = environment;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {


      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
