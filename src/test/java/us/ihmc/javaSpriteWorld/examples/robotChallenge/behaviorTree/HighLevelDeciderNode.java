package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class HighLevelDeciderNode implements BehaviorTreeAction
{
   private BehaviorStatusHolder statusHolder;
   private RobotBehaviorActuators actuators;

   public HighLevelDeciderNode(BehaviorStatusHolder statusHolder, RobotBehaviorActuators actuators)
   {
      this.statusHolder = statusHolder;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (statusHolder.getWallWeight() > 0.5)
      {
         double[] wallAction = statusHolder.getWallAction();
         actuators.setAcceleration(wallAction[0]);
         actuators.setTurnRate(wallAction[1]);
      }
      else if (statusHolder.getPredatorWeight() > 0.5)
      {
         double[] predatorAction = statusHolder.getPredatorAction();
         actuators.setAcceleration(predatorAction[0]);
         actuators.setTurnRate(predatorAction[1]);
      }
      else if (statusHolder.getHungerWeight() > 0.5)
      {
         double[] hungerAction = statusHolder.getHungerAction();
         actuators.setAcceleration(hungerAction[0]);
         actuators.setTurnRate(hungerAction[1]);
      }
      else
      {
         double[] flagAction = statusHolder.getFlagAction();
         actuators.setAcceleration(flagAction[0]);
         actuators.setTurnRate(flagAction[1]);
      }

      actuators.setDropFlag(statusHolder.isDropFlag());

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
