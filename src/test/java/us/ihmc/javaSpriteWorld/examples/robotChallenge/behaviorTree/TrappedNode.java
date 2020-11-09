package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class TrappedNode implements BehaviorTreeAction
{
   private BehaviorStatusHolder statusHolder;
   private RobotBehaviorActuators actuators;

   public TrappedNode(BehaviorStatusHolder statusHolder, RobotBehaviorActuators actuators)
   {
      this.statusHolder = statusHolder;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (statusHolder.isTrapped())
      {
         statusHolder.setTrapped(false);

         // do a persistent travel backwards thing
      }

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
