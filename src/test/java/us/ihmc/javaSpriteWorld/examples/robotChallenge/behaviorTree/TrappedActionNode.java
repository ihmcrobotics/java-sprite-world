package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class TrappedActionNode implements BehaviorTreeAction
{
   private RobotBehaviorSensors sensors;
   private BehaviorStatusHolder statusHolder;
   private RobotBehaviorActuators actuators;
   private double lastHeading = 0.0;
   private boolean trappedInitialized = false;

   public TrappedActionNode(RobotBehaviorSensors sensors, BehaviorStatusHolder statusHolder, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.statusHolder = statusHolder;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (statusHolder.isTrapped())
      {
         if (!trappedInitialized)
         {
            trappedInitialized = true;

            lastHeading = sensors.getHeading();
         }

         // do a persistent travel backwards thing

         double elapsedTrappedTime = statusHolder.getTrappedTime();

         double desiredSpeed;
         double angleToAttraction;
         double headingError = sensors.getHeading() - statusHolder.getTrappedHeading();
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

         return BehaviorTreeNodeStatus.RUNNING;
      }
      else
      {
         trappedInitialized = false;
      }

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
