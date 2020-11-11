package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNode;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.log.LogTools;

public class TrappedEvaluationNode implements BehaviorTreeNode
{
   private RobotBehaviorSensors sensors;
   private BehaviorStatusHolder statusHolder;

   public TrappedEvaluationNode(RobotBehaviorSensors sensors, BehaviorStatusHolder statusHolder)
   {
      this.sensors = sensors;
      this.statusHolder = statusHolder;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (statusHolder.getPredatorWeight() > 0.5
          && statusHolder.getWallWeight() > 0.5
          && !calculateSimilar(statusHolder.getWallAction(), statusHolder.getPredatorAction()))
      {
         statusHolder.setTrapped(System.nanoTime());
         LogTools.info("Trapped!");
      }

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   private boolean calculateSimilar(double[] actionA, double[] actionB)
   {
      Action a = new Action(actionA);
      Action b = new Action(actionB);

      if (a.getAcceleration() > 0.1 && b.getAcceleration() < -0.1)
      {
         return false;
      }
      if (a.getTurnRate() > 0.1 && b.getTurnRate() < -0.1)
      {
         return false;
      }

      return true;
   }

   private double calculateSimilarity(double[] actionA, double[] actionB)
   {
      double accelerationWeight = 0.5;
      double turnRateWeight = 0.5;
      double accelerationDifference = Math.abs(actionA[0] - actionB[0]);
      double turnRateDifference = Math.abs(actionA[1] - actionB[1]);

      double activationThreshold = 0.1;
      if (actionA[0] < 0.1 && actionB[0] < 0.1 && actionA[1] < 0.1 && actionB[1] < activationThreshold)
      {
         return 1.0;
      }

      double maxAcceleration = Math.max(Math.abs(actionA[0]), Math.abs(actionB[0]));
      double maxTurnRate = Math.max(Math.abs(actionA[1]), Math.abs(actionB[1]));

      double accelerationSimilarity = accelerationWeight * maxAcceleration / accelerationDifference;
      double turnRateSimilarity = turnRateWeight * maxTurnRate / turnRateDifference;

      return accelerationSimilarity + turnRateSimilarity;

      // dot product
      //      return actionA[0] * actionB[0] + actionA[1] * actionB[1];
   }

   private class Action
   {
      private final double acceleration;
      private final double turnRate;

      public Action(double[] action)
      {
         this.acceleration = action[0];
         this.turnRate = action[1];
      }

      public double getAcceleration()
      {
         return acceleration;
      }

      public double getTurnRate()
      {
         return turnRate;
      }
   }
}