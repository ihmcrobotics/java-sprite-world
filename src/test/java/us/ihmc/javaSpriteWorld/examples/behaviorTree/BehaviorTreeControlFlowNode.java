package us.ihmc.javaSpriteWorld.examples.behaviorTree;

/**
 * A behavior tree control flow node can have children, so provide the ability to add them.
 */
public interface BehaviorTreeControlFlowNode extends BehaviorTreeNode
{
   <T extends BehaviorTreeNode> T addChild(T child);
}
