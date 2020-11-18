package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorActuators;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorEnvironment;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorSensors;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import java.util.Random;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.*;

public class SearchForFlagBehaviorNode implements BehaviorTreeAction
{
   private final Random random = new Random(3920);

   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final FlagBehaviorBlackBoard flagBehaviorBlackBoard;

   private int counter = 0;
   private final int counterToSwitchExploreArea = 500;
   private final double proximityToExploreAreaToSwitch = 0.4;

   private final Point2D[] areasToExplore;
   private int indexToExplore;

   private final double[] action = new double[2];

   public SearchForFlagBehaviorNode(RobotBehaviorSensors sensors,
                                    RobotBehaviorActuators actuators,
                                    FlagBehaviorBlackBoard flagBehaviorBlackBoard,
                                    RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.flagBehaviorBlackBoard = flagBehaviorBlackBoard;

      areasToExplore = new Point2D[4];

      double dSmall = environment.getMapSizeX() * 0.2;
      double dLarge = environment.getMapSizeX() * 0.8;

      areasToExplore[0] = new Point2D(dSmall, dLarge);
      areasToExplore[1] = new Point2D(dLarge, dLarge);
      areasToExplore[2] = new Point2D(dLarge, dSmall);
      areasToExplore[3] = new Point2D(dSmall, dSmall);
   }

   @Override
   public double evaluateUtility()
   {
      handleCommonFlagStuff();

      boolean hasSeenNextFlag = flagBehaviorBlackBoard.hasSeenNextFlag();
      if (hasSeenNextFlag)
         return 0.0;

      return 0.4;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      updateAreaToExplore();
      Point2D areaToExploreInBodyFrame = new Point2D();
      worldFrameToBodyFrame(areasToExplore[indexToExplore], areaToExploreInBodyFrame, sensors.getHeading(), sensors.getGlobalPosition());
      double heading = headingFromVector(areaToExploreInBodyFrame);

      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 3.0;
      SteeringBasedAction.computeActionGivenHeading(action, heading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());

      actuators.setAcceleration(action[0]);
      actuators.setTurnRate(action[1]);
      actuators.setDropFlag(false);

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   private void updateAreaToExplore()
   {
      boolean counterReachedLimit = counter++ >= counterToSwitchExploreArea;
      boolean closeToExplorationArea = areasToExplore[indexToExplore].distance(sensors.getGlobalPosition()) < proximityToExploreAreaToSwitch;

      if (counterReachedLimit || closeToExplorationArea)
      {
         indexToExplore = (indexToExplore + 1) % areasToExplore.length;
         counter = 0;
      }
   }

   private void handleCommonFlagStuff()
   {
      if (sensors.isSimulationReset())
      {
         flagBehaviorBlackBoard.reset();
         indexToExplore = 0;
      }

      bodyFrameToWorldFrame(sensors.getPositionInBodyFrameAndIdOfClosestFlag().getLeft(),
                            flagBehaviorBlackBoard.getFlagLocation(sensors.getPositionInBodyFrameAndIdOfClosestFlag().getRight()),
                            sensors.getHeading(),
                            sensors.getGlobalPosition());

      int newPickedUpFlag = sensors.pollSensedPickedUpFlag();
      if (newPickedUpFlag != -1)
      {
         flagBehaviorBlackBoard.setHasCorrectFlag(newPickedUpFlag == flagBehaviorBlackBoard.getFlagIdToChase());
         flagBehaviorBlackBoard.getFlagLocation(newPickedUpFlag).setToNaN();
      }

      int deliveredFlagId = sensors.pollSensedDeliveredFlag();
      if (deliveredFlagId != -1)
      {
         flagBehaviorBlackBoard.deliveredFlag();
      }

   }
}
