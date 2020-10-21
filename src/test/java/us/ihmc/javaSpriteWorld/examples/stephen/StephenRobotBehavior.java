package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;

import java.util.ArrayList;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.filter;

public class StephenRobotBehavior implements Robot05Behavior, Robot06Behavior
{
   private static final boolean useSteeringAction = true;

   private final EnabledBehaviors enabledBehaviors = new EnabledBehaviors();
   private final SLAMManager slamManager = new SLAMManager();
   private final FlagManager flagManager = new FlagManager(slamManager);
   private final RadialVectorAction radialVectorAction = new RadialVectorAction(slamManager, flagManager, enabledBehaviors);
   private final SteeringBasedAction steeringBasedAction = new SteeringBasedAction(slamManager, flagManager, enabledBehaviors);

   private boolean firstTick = true;
   private final double[] totalAction = new double[2];
   private final double[] previousAction = new double[2];
   private final double[] filteredAction = new double[2];

   // filter parameters
   private final double alphaAction = 1.0;
   private final double alphaVelocity = 1.0;

   public StephenRobotBehavior()
   {
      enabledBehaviors.setFoodEnabled(true);
      enabledBehaviors.setWallEnabled(false);
      enabledBehaviors.setPredatorEnabled(false);
      enabledBehaviors.setFlagEnabled(false);
   }

   @Override
   public void senseVelocity(double velocity)
   {
      slamManager.setVelocity(filter(alphaVelocity, velocity, slamManager.getVelocity()));
   }

   @Override
   public void senseHeading(double heading)
   {
      slamManager.setHeading(heading);
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {

   }

   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      slamManager.setVectorsAndDistancesToWallInBodyFrame(vectorsAndDistancesToWallInBodyFrame);
      radialVectorAction.senseWallRangeInBodyFrame(vectorsAndDistancesToWallInBodyFrame);
      steeringBasedAction.senseWallRangeInBodyFrame(vectorsAndDistancesToWallInBodyFrame);
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {

   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      slamManager.senseGlobalPositionForTestingOnly(x, y);
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      radialVectorAction.senseFoodInBodyFrame(locationOfAllFood);
      steeringBasedAction.senseFoodInBodyFrame(locationOfAllFood);
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      radialVectorAction.sensePredatorsInBodyFrame(locationOfAllPredators);
      steeringBasedAction.sensePredatorsInBodyFrame(locationOfAllPredators);
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      flagManager.senseClosestFlagInBodyFrame(positionInBodyFrameAndIdOfClosestFlag);
      radialVectorAction.senseClosestFlagInBodyFrame(positionInBodyFrameAndIdOfClosestFlag);
      steeringBasedAction.senseClosestFlagInBodyFrame(positionInBodyFrameAndIdOfClosestFlag);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      slamManager.update();
      flagManager.update();

      if (useSteeringAction)
      {
         steeringBasedAction.computeAction(totalAction);
      }
      else
      {
         radialVectorAction.computeAction(totalAction);
      }

      if (firstTick)
      {
         firstTick = false;
         for (int i = 0; i < 2; i++)
         {
            filteredAction[i] = totalAction[i];
         }
      }
      else
      {
         for (int i = 0; i < 2; i++)
         {
            filteredAction[i] = filter(alphaAction, totalAction[i], previousAction[i]);
         }
      }

      for (int i = 0; i < 2; i++)
      {
         previousAction[i] = totalAction[i];
      }

      return filteredAction;
   }

   public SLAMManager getSlamManager()
   {
      return slamManager;
   }

   @Override
   public void senseNoiseFreeHeadingForTestingOnly(double heading)
   {

   }


   @Override
   public boolean getDropFlag()
   {
      return flagManager.getDropFlag();
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      System.out.println("droppedFlag " + flagId);
   }

   @Override
   public void senseHitWall()
   {
      // TODO reset SLAM
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      flagManager.sensePickedUpFlag(id);
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      flagManager.senseDeliveredFlag(flagId);
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double elapsedTime)
   {

   }

   @Override
   public void reset()
   {

   }

   @Override
   public void senseCarryingFlag(int flagId)
   {

   }
}
