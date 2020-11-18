package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorActuators;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorSensors;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;
import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.worldFrameToBodyFrame;

public class GoToFlagBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final FlagBehaviorBlackBoard flagBehaviorBlackBoard;

   private final double[] action = new double[2];

   public GoToFlagBehaviorNode(RobotBehaviorSensors sensors,
                                    RobotBehaviorActuators actuators,
                                    FlagBehaviorBlackBoard flagBehaviorBlackBoard)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.flagBehaviorBlackBoard = flagBehaviorBlackBoard;
   }

   @Override
   public double evaluateUtility()
   {
      if (flagBehaviorBlackBoard.hasSeenNextFlag() && !flagBehaviorBlackBoard.hasCorrectFlag())
      {
         return 0.6;
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      Point2D nextFlagInBody = new Point2D();
      worldFrameToBodyFrame(flagBehaviorBlackBoard.getNextFlagLocation(), nextFlagInBody, sensors.getHeading(), sensors.getGlobalPosition());
      double heading = headingFromVector(nextFlagInBody);

      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 3.0;
      SteeringBasedAction.computeActionGivenHeading(action, heading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());

      actuators.setAcceleration(action[0]);
      actuators.setTurnRate(action[1]);
      actuators.setDropFlag(false);

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
