package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class GetFoodBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;

   public GetFoodBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {

      return BehaviorTreeNodeStatus.RUNNING;
   }

}