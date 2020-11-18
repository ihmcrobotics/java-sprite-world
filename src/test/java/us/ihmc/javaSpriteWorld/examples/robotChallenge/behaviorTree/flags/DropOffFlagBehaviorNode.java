package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorActuators;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorEnvironment;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorSensors;

public class DropOffFlagBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final FlagBehaviorBlackBoard flagBehaviorBlackBoard;
   private final RobotBehaviorEnvironment environment;

   private final double[] action = new double[2];

   public DropOffFlagBehaviorNode(RobotBehaviorSensors sensors,
                                  RobotBehaviorActuators actuators,
                                  FlagBehaviorBlackBoard flagBehaviorBlackBoard,
                                  RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.flagBehaviorBlackBoard = flagBehaviorBlackBoard;
      this.environment = environment;
   }

   @Override
   public double evaluateUtility()
   {
      boolean inDropOffZone =
            sensors.getGlobalPosition().getX() > (0.85 * environment.getMapSizeX()) && sensors.getGlobalPosition().getY() > (0.85 * environment.getMapSizeY());

      if (flagBehaviorBlackBoard.hasCorrectFlag() && inDropOffZone)
      {
         return 1.0;
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      actuators.setDropFlag(true);
      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
