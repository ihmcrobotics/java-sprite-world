package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorActuators;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorSensors;

public class AvoidFlagBehaviorNode implements BehaviorTreeAction
{
   private static final double PROXIMITY_TO_AVOID_FLAG_WHEN_DIRECTLY_IN_FRONT = 1.0;

   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final FlagBehaviorBlackBoard flagBehaviorBlackBoard;

   public AvoidFlagBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators, FlagBehaviorBlackBoard flagBehaviorBlackBoard)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.flagBehaviorBlackBoard = flagBehaviorBlackBoard;
   }

   @Override
   public double evaluateUtility()
   {
      return 0.0;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      return null;
   }
}
