package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

import static us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public class ZeroMotionAction implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private double lastHeading;

   public ZeroMotionAction(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      double desiredSpeed = 0.0;
      double angleToAttraction = 0.0;

      double velocity = sensors.getVelocity();
      double acceleration = 3.0 * (desiredSpeed - velocity);
      actuators.setAcceleration(acceleration);

      double dt = 0.01;
      double headingChange = sensors.getHeading() - lastHeading;
      double angularVelocity = headingChange / dt;
      lastHeading = sensors.getHeading();

      double turnRate = (8.0 * angleToAttraction) + (-0.2 * angularVelocity);
      actuators.setTurnRate(turnRate);

      return SUCCESS;
   }
}
