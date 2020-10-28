package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class AvoidWallsBehaviorAction implements BehaviorTreeAction
{
   private RobotBehaviorSensors sensors;
   private RobotBehaviorActuators actuators;

   public AvoidWallsBehaviorAction(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {


      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
