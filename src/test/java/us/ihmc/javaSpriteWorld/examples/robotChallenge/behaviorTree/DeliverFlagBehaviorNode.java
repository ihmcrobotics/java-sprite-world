package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.stephen.ObjectResponseDescription;
import us.ihmc.javaSpriteWorld.examples.stephen.RampedAngularReward;
import us.ihmc.javaSpriteWorld.examples.stephen.SteeringBasedAction;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.*;

public class DeliverFlagBehaviorNode implements BehaviorTreeAction
{
   private static final Random random = new Random(3920);

   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private final RobotBehaviorEnvironment environment;

   private final List<ObjectResponseDescription> responseDescriptions = new ArrayList<>();

   private final double avoidFlagWeight = 2.5;
   private final double gotoFlagWeight = 0.7;
   private final double exploreAreaWeight = 0.4;
   private final double baseAvoidFlag = 1.75;

   private final double goToFlagAngularCostRange = Math.toRadians(180.0);
   private final double avoidFlagAngularCostRange = Math.toRadians(130.0);

   private int flagIdToChase = 1;
   private boolean inDeliverFlagMode = false;
   private final Point2D[] flagLocations;

   private final Point2D[] areasToExplore;
   private final Point2D areaToExplore = new Point2D();

   private int counter = 0;
   private final int counterToSwitchExploreArea = 500;
   private final double proximityToExploreAreaToSwitch = 0.4;

   private boolean hasFlag;
   private boolean dropFlag;
   private final double[] flagAction = new double[2];

   public DeliverFlagBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators, RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.environment = environment;

      flagLocations = new Point2D[sensors.getNumberOfFlags()];

      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i] = new Point2D();
         flagLocations[i].setToNaN();
      }

      areasToExplore = new Point2D[9];
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            areasToExplore[3 * i + j] = new Point2D((0.5 * environment.getMapSizeX()) + (0.3 * environment.getMapSizeX()) * (i - 1),
                                                    (0.5 * environment.getMapSizeY()) + (0.3 * environment.getMapSizeY()) * (j - 1));
         }
      }

      switchAreaToExplore();
   }

   private void switchAreaToExplore()
   {
      while (true)
      {
         Point2D newAreaToExplore = areasToExplore[random.nextInt(areasToExplore.length)];
         if (newAreaToExplore.equals(areaToExplore))
         {
            continue;
         }
         else
         {
            areaToExplore.set(newAreaToExplore);
            return;
         }
      }
   }

   @Override
   public double evaluateUtility()
   {
      if (sensors.isSimulationReset())
      {
         reset();
      }

      if (sensors.getPositionInBodyFrameAndIdOfClosestFlag() == null)
      {
         return 0.0;
      }

      responseDescriptions.clear();
      takeANoteOfFlagLocation();
      updateAreaToExplore();

      int newPickedUpFlag = sensors.pollSensedPickedUpFlag();
      if (newPickedUpFlag != -1)
      {
         if (newPickedUpFlag == flagIdToChase)
         {
            inDeliverFlagMode = true;
         }
         else if (inDeliverFlagMode)
         {
            inDeliverFlagMode = false;
         }

         flagLocations[newPickedUpFlag - 1].setToNaN();
      }

      int deliveredFlagId = sensors.pollSensedDeliveredFlag();
      if (deliveredFlagId != -1)
      {
         if (inDeliverFlagMode)
         {
            if (flagIdToChase == 5)
            {
               flagIdToChase = 1;
            }
            else
            {
               flagIdToChase++;
            }

            inDeliverFlagMode = false;
         }
         else
         {
            throw new RuntimeException("Shouldn't get here...");
         }
      }

      if (inDeliverFlagMode)
      {
         Point2D dropPointWorld = new Point2D(10.0, 10.0);
         Point2D dropPointBody = new Point2D();
         worldFrameToBodyFrame(dropPointWorld, dropPointBody, sensors.getHeading(), sensors.getGlobalPosition());

         computeGoToFlagReward(dropPointBody, gotoFlagWeight);

         if (flagIdToChase != 5)
         {
            computeAvoidFlagPenalty(sensors.getPositionInBodyFrameAndIdOfClosestFlag().getLeft());
         }
      }
      else
      {
         boolean detectedFlagShouldBeRetrieved = sensors.getPositionInBodyFrameAndIdOfClosestFlag().getRight() == flagIdToChase;
         if (detectedFlagShouldBeRetrieved)
         {
            computeGoToFlagReward(sensors.getPositionInBodyFrameAndIdOfClosestFlag().getLeft(), gotoFlagWeight);
         }
         else
         {
            Point2D nextFlagLocationInWorld = flagLocations[flagIdToChase - 1];
            if (!nextFlagLocationInWorld.containsNaN())
            {
               Point2D nextFlagLocationInBodyFrame = new Point2D();
               worldFrameToBodyFrame(nextFlagLocationInWorld, nextFlagLocationInBodyFrame, sensors.getHeading(), sensors.getGlobalPosition());
               computeGoToFlagReward(nextFlagLocationInBodyFrame, gotoFlagWeight);
            }
            else
            {
               Point2D areaToExploreInBodyFrame = new Point2D();
               worldFrameToBodyFrame(areaToExplore, areaToExploreInBodyFrame, sensors.getHeading(), sensors.getGlobalPosition());
               computeGoToFlagReward(areaToExploreInBodyFrame, exploreAreaWeight);
            }
         }
      }

      double maxRewardHeading = SteeringBasedAction.getMaxRewardHeading(responseDescriptions);

      double velocityWhenAligned = 3.0;
      double kAcceleration = 3.0;
      double kTurn = 4.0;

      hasFlag = inDeliverFlagMode;
      SteeringBasedAction.computeActionGivenHeading(flagAction, maxRewardHeading, velocityWhenAligned, kAcceleration, kTurn, sensors.getVelocity());
      dropFlag = getDropFlag();

      actuators.setDropFlag(dropFlag);

      double utility = 0.9;
      return utility;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      actuators.setAcceleration(flagAction[0]);
      actuators.setTurnRate(flagAction[1]);

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   private void computeGoToFlagReward(Tuple2DReadOnly flagInBodyFrame, double reward)
   {
      double deadband = 0.5;
      if (EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY()) < deadband)
      {
         responseDescriptions.add(new RampedAngularReward(0.0, goToFlagAngularCostRange, reward));
      }
      else
      {
         responseDescriptions.add(new RampedAngularReward(headingFromVector(flagInBodyFrame), goToFlagAngularCostRange, reward));
      }
   }

   private void computeAvoidFlagPenalty(Tuple2DReadOnly flagInBodyFrame)
   {
      double heading = headingFromVector(flagInBodyFrame);
      double distance = EuclidCoreTools.norm(flagInBodyFrame.getX(), flagInBodyFrame.getY());
      double cost = -avoidFlagWeight * Math.pow(baseAvoidFlag, - distance);
      responseDescriptions.add(new RampedAngularReward(heading, avoidFlagAngularCostRange, cost));
   }

   private void updateAreaToExplore()
   {
      if (counter++ >= counterToSwitchExploreArea)
      {
         switchAreaToExplore();
         counter = 0;
      }
      else if (areaToExplore.distance(sensors.getGlobalPosition()) < proximityToExploreAreaToSwitch)
      {
         switchAreaToExplore();
         counter = 0;
      }
   }

   private void takeANoteOfFlagLocation()
   {
      int flagZeroIndexId = sensors.getPositionInBodyFrameAndIdOfClosestFlag().getRight() - 1;
      bodyFrameToWorldFrame(sensors.getPositionInBodyFrameAndIdOfClosestFlag().getLeft(),
                            flagLocations[flagZeroIndexId],
                            sensors.getHeading(),
                            sensors.getGlobalPosition());
   }

   public boolean getDropFlag()
   {
      return inDeliverFlagMode
             && (sensors.getGlobalPosition().getX() > (0.8 * environment.getMapSizeX())
             && (sensors.getGlobalPosition().getY() > (0.8 * environment.getMapSizeY())));
   }

   public void reset()
   {
      inDeliverFlagMode = false;
      flagIdToChase = 1;

      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i].setToNaN();
      }

      counter = 0;
   }
}
