package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.FallbackNode;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.UtilitySelectorNode;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;

public class BehaviorTreeBehavior implements Robot05Behavior
{
   private final FallbackNode fallbackNode = new FallbackNode();
   private final UtilitySelectorNode utilitySelector  = new UtilitySelectorNode();
   private final RobotBehaviorActuators actuators = new RobotBehaviorActuators();
   private final RobotBehaviorSensors sensors = new RobotBehaviorSensors();

   public BehaviorTreeBehavior()
   {
      int challengeNumber = 5;
      RobotBehaviorEnvironment environment = new RobotBehaviorEnvironment(challengeNumber);

      AvoidWallsBehaviorNode avoidWalls = new AvoidWallsBehaviorNode(sensors, actuators, environment);
      AvoidPredatorsBehaviorNode avoidPredators = new AvoidPredatorsBehaviorNode(sensors, actuators);
      GetFoodBehaviorNode getFood = new GetFoodBehaviorNode(sensors, actuators);
      GoForwardBehaviorNode goForward = new GoForwardBehaviorNode(sensors, actuators);
      DeliverFlagBehaviorNode deliverFlag = new DeliverFlagBehaviorNode(sensors, actuators);
      TrappedActionNode trappedAction = new TrappedActionNode(sensors, actuators);
      ZeroMotionAction zeroMotion = new ZeroMotionAction(sensors, actuators);

      utilitySelector.addChild(avoidWalls);
      utilitySelector.addChild(getFood);
      utilitySelector.addChild(avoidPredators);
      utilitySelector.addChild(deliverFlag);

      fallbackNode.addChild(utilitySelector);
      fallbackNode.addChild(zeroMotion);
   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      sensors.setGlobalPosition(x, y);
   }

   @Override
   public void senseNoiseFreeHeadingForTestingOnly(double heading)
   {
      sensors.setHeading(heading);      
   }

   @Override
   public void senseVelocity(double velocity)
   {
      sensors.setVelocity(velocity);
   }

   @Override
   public void senseHeading(double heading)
   {
      sensors.setHeading(heading);
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {      
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {      
   }

   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      sensors.setVectorsAndDistancesToWallInBodyFrame(vectorsAndDistancesToWallInBodyFrame);
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      sensors.setLocationOfAllFood(locationOfAllFood);
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      sensors.setLocationOfAllPredators(locationOfAllPredators);
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      sensors.setPositionInBodyFrameAndIdOfClosestFlag(positionInBodyFrameAndIdOfClosestFlag);
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      sensors.senseDroppedFlag(flagId);
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      sensors.sensePickedUpFlag(id);
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      sensors.senseDeliveredFlag(flagId);
   }

   @Override
   public void senseCarryingFlag(int flagId)
   {
      sensors.senseCarryingFlag(flagId);
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      fallbackNode.tick();
      sensors.clearSimulationReset();
      return new double[] {actuators.getAcceleration(), actuators.getTurnRate()};
   }

   @Override
   public boolean getDropFlag()
   {
      return actuators.getDropFlag();
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double elapsedTime)
   {
      sensors.setScore(score);
      sensors.setHealth(health);
      sensors.setElapsedTime(elapsedTime);
   }

   @Override
   public void reset()
   {
      sensors.setSimulationReset();
   }

   @Override
   public void exit()
   {
      // TODO Auto-generated method stub
      
   }
}
