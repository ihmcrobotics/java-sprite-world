package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorActuators;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorEnvironment;
import us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorSensors;
import us.ihmc.javaSpriteWorld.examples.stephen.ObjectResponseDescription;
import us.ihmc.javaSpriteWorld.examples.stephen.RampedAngularReward;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.headingFromVector;
import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.worldFrameToBodyFrame;

public class GoToFlagDropOffBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final FlagBehaviorBlackBoard flagBehaviorBlackBoard;
   private final RobotBehaviorEnvironment environment;

   private final double avoidFlagWeight = 1.75;
   private final double goToDropOffWeight = 0.7;
   private final double baseAvoidFlag = 1.75;
   private double goToDropOffAngularRange = Math.toRadians(180.0);
   private double avoidFlagAngularRange = Math.toRadians(130.0);

   private final double[] action = new double[2];

   public GoToFlagDropOffBehaviorNode(RobotBehaviorSensors sensors,
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

      if (flagBehaviorBlackBoard.hasCorrectFlag() && !inDropOffZone)
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
      Point2D dropPointWorld = new Point2D(10.0, 10.0);
      Point2D dropPointBody = new Point2D();
      worldFrameToBodyFrame(dropPointWorld, dropPointBody, sensors.getHeading(), sensors.getGlobalPosition());
      double dropOffHeading = headingFromVector(dropPointBody);

      double deadband = 0.5;
      List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();
      if (EuclidCoreTools.norm(dropPointBody.getX(), dropPointBody.getY()) < deadband)
      {
         responseDescriptions.add(new RampedAngularReward(0.0, goToDropOffAngularRange, goToDropOffWeight));
      }
      else
      {
         responseDescriptions.add(new RampedAngularReward(dropOffHeading, goToDropOffAngularRange, goToDropOffWeight));
      }

      for (int i = 0; i < 5; i++)
      {
         int flagNumber = i + 1;
         Point2D flagLocation = flagBehaviorBlackBoard.getFlagLocation(flagNumber);
         if (flagLocation.containsNaN() || flagBehaviorBlackBoard.getFlagIdToChase() == flagNumber)
            continue;

         double headingOfFlagToAvoid = headingFromVector(flagLocation);
         double distance = EuclidCoreTools.norm(flagLocation.getX(), flagLocation.getY());
         double cost = -avoidFlagWeight * Math.pow(baseAvoidFlag, - distance);
         responseDescriptions.add(new RampedAngularReward(headingOfFlagToAvoid, avoidFlagAngularRange, cost));
      }

      double maxRewardHeading = SteeringBasedAction.getMaxRewardHeading(responseDescriptions);
      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 3.0;

      SteeringBasedAction.computeActionGivenHeading(action, maxRewardHeading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());

      actuators.setAcceleration(action[0]);
      actuators.setTurnRate(action[1]);
      actuators.setDropFlag(false);

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
