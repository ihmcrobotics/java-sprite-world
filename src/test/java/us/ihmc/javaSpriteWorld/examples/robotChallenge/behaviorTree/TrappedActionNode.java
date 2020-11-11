package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.commons.Conversions;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.log.LogTools;

public class TrappedActionNode implements BehaviorTreeAction
{
   private RobotBehaviorSensors sensors;
   private BehaviorStatusHolder statusHolder;
   private RobotBehaviorActuators actuators;
   private double lastAngularVelocity = 0.0;

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
         // do a persistent travel backwards thing

         double elapsedTrappedTime = Conversions.nanosecondsToSeconds(System.nanoTime() - statusHolder.getTrappedTime());

         double desiredSpeed;
         double angleToAttraction;
         double headingError = sensors.getHeading() - statusHolder.getTrappedHeading();
         if (elapsedTrappedTime < 0.3) // slow down
         {
            desiredSpeed = 0.0;
            angleToAttraction = -headingError;
            LogTools.info("Slowing down");
         }
         else if (elapsedTrappedTime < 0.6) // turn around
         {
            desiredSpeed = 0.0;
            angleToAttraction = -headingError - Math.PI;
            LogTools.info("Turning around");
         }
         else // go backwards
         {
            desiredSpeed = 1.0;
            angleToAttraction = -headingError - Math.PI;
            LogTools.info("Going backwards");
         }

         double velocity = sensors.getVelocity();
         actuators.setAcceleration(1.0 * (desiredSpeed - velocity));

         double angularVelocity = (velocity - lastAngularVelocity) / 0.01;
         actuators.setTurnRate((5.0 * angleToAttraction) + (-0.5 * angularVelocity));

         return BehaviorTreeNodeStatus.RUNNING;
      }

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
