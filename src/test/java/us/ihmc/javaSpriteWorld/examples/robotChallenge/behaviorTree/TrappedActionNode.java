package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class TrappedActionNode implements BehaviorTreeAction
{
   private RobotBehaviorSensors sensors;
   private RobotBehaviorActuators actuators;
   private double lastHeading = 0.0;
   private boolean trappedInitialized = false;

   private double trappedHeading;
   private Stopwatch trappedStopwatch = new Stopwatch();

   public TrappedActionNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public double evaluateUtility()
   {
//      if (!isTrapped()
//          && statusHolder.getPredatorWeight() > 0.5
//          && statusHolder.getWallWeight() > 0.5
//          && !calculateSimilar(statusHolder.getWallAction(), statusHolder.getPredatorAction()))
//      {
//         statusHolder.setTrapped(sensors.getHeading());
//         LogTools.info("Trapped!");
//      }
//
//      if (!isTrapped())
//      {
//         trappedInitialized = false;
//      }

      double utility = isTrapped() ? 1.0 : 0.0;

      return 0.0;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (!trappedInitialized)
      {
         trappedInitialized = true;

         lastHeading = sensors.getHeading();
      }

      // do a persistent travel backwards thing

      double elapsedTrappedTime = trappedStopwatch.totalElapsed();

      double desiredSpeed;
      double angleToAttraction;
      double headingError = sensors.getHeading() - trappedHeading;
      if (elapsedTrappedTime < 0.3) // slow down
      {
         desiredSpeed = 0.0;
         angleToAttraction = -headingError;
      }
      else if (elapsedTrappedTime < 0.6) // turn around
      {
         desiredSpeed = 0.0;
         angleToAttraction = -headingError - Math.PI;
      }
      else // go backwards
      {
         desiredSpeed = 8.0;
         angleToAttraction = -headingError - Math.PI;
      }

      double velocity = sensors.getVelocity();
      double acceleration = 3.0 * (desiredSpeed - velocity);
      actuators.setAcceleration(acceleration);

      double dt = 0.01;
      double headingChange = sensors.getHeading() - lastHeading;
      double angularVelocity = headingChange / dt;
      lastHeading = sensors.getHeading();

      double turnRate = (8.0 * angleToAttraction) + (-0.2 * angularVelocity);
      actuators.setTurnRate(turnRate);

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   private boolean isTrapped()
   {
      return trappedStopwatch.totalElapsed() < 1.2;
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
