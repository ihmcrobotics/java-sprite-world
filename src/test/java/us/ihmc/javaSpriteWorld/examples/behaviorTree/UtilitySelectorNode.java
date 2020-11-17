package us.ihmc.javaSpriteWorld.examples.behaviorTree;

public class UtilitySelectorNode extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public BehaviorTreeNodeStatus tick()
   {
      double highestUtility = 0.0;
      BehaviorTreeNode nodeOfHighestUtility = null;
      for (BehaviorTreeNode child : getChildren())
      {
         double utility = child.evaluateUtility();
         if (utility > highestUtility)
         {
            highestUtility = utility;
            nodeOfHighestUtility = child;
         }
      }

      if (highestUtility > 0.0)
      {
         return nodeOfHighestUtility.tick();
      }
      else
      {
         return BehaviorTreeNodeStatus.FAILURE;
      }
   }
}
