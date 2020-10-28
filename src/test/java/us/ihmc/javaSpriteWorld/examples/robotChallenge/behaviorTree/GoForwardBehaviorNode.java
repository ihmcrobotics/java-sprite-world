package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class GoForwardBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;

   public GoForwardBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      double desiredVelocity = 10.0;
      
      double acceleration = 1.0 * (desiredVelocity - sensors.getVelocity());
      actuators.setAcceleration(acceleration);
      actuators.setTurnRate(0.0);
      
      return BehaviorTreeNodeStatus.RUNNING;
   }
}

