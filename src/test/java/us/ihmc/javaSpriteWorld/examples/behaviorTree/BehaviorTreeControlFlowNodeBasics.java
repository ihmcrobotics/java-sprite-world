package us.ihmc.javaSpriteWorld.examples.behaviorTree;

import java.util.ArrayList;

/**
 * Add default ArrayList storage of children nodes for a control flow node.
 */
public abstract class BehaviorTreeControlFlowNodeBasics implements BehaviorTreeControlFlowNode
{
   private final ArrayList<BehaviorTreeNode> children = new ArrayList<>();

   protected ArrayList<BehaviorTreeNode> getChildren()
   {
      return children;
   }

   @Override
   public <T extends BehaviorTreeNode> T addChild(T child)
   {
      children.add(child);
      return child;
   }
}
