package us.ihmc.javaSpriteWorld.examples.behaviorTree;

import static us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A fallback node proceeds through children left to right until they return SUCCESS.
 */
public class FallbackNode extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public BehaviorTreeNodeStatus tick()
   {
      for (BehaviorTreeNode child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNode.checkStatusInNotNull(childStatus);

         if (childStatus == RUNNING)
         {
            return RUNNING;
         }
         else if (childStatus == SUCCESS)
         {
            return SUCCESS;
         }

         // FAILURE, continue
      }

      return FAILURE;
   }
}
