package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.commons.Conversions;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.log.LogTools;

public class TrappedActionNode implements BehaviorTreeAction
{
   private BehaviorStatusHolder statusHolder;
   private RobotBehaviorActuators actuators;

   public TrappedActionNode(BehaviorStatusHolder statusHolder, RobotBehaviorActuators actuators)
   {
      this.statusHolder = statusHolder;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (System.nanoTime() - statusHolder.getTrappedTime() < Conversions.secondsToNanoseconds(1.0))
      {
         // TODO
         // do a persistent travel backwards thing
      }

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
