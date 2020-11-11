package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.SequenceNode;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;

public class BehaviorTreeBehavior implements Robot05Behavior
{
   private final SequenceNode behaviorTree;
   private final SequenceNode evaluationTree;
   private final SequenceNode decisionTree;
   private final RobotBehaviorActuators actuators = new RobotBehaviorActuators();
   private final RobotBehaviorSensors sensors = new RobotBehaviorSensors();

   private final DeliverFlagBehaviorNode deliverFlag;

   public BehaviorTreeBehavior()
   {
      behaviorTree = new SequenceNode();
      evaluationTree = new SequenceNode();
      decisionTree = new SequenceNode();
      behaviorTree.addChild(evaluationTree);
      behaviorTree.addChild(decisionTree);

      int challengeNumber = 5;
      RobotBehaviorEnvironment environment = new RobotBehaviorEnvironment(challengeNumber);

      BehaviorStatusHolder statusHolder = new BehaviorStatusHolder();

      TrappedEvaluationNode trappedEvaluation = new TrappedEvaluationNode();
      AvoidWallsBehaviorNode avoidWalls = new AvoidWallsBehaviorNode(sensors, environment, statusHolder);
      AvoidPredatorsBehaviorNode avoidPredators = new AvoidPredatorsBehaviorNode(sensors, statusHolder);
      GetFoodBehaviorNode getFood = new GetFoodBehaviorNode(sensors, statusHolder);
      GoForwardBehaviorNode goForward = new GoForwardBehaviorNode(sensors, actuators);
      deliverFlag = new DeliverFlagBehaviorNode(sensors, statusHolder);
      HighLevelDeciderNode highLevelDecider = new HighLevelDeciderNode(statusHolder, actuators);
      TrappedActionNode trappedAction = new TrappedActionNode(statusHolder, actuators);

      evaluationTree.addChild(trappedEvaluation);
      evaluationTree.addChild(avoidWalls);
      evaluationTree.addChild(avoidPredators);
      evaluationTree.addChild(getFood);
      evaluationTree.addChild(deliverFlag);

      decisionTree.addChild(highLevelDecider);
      decisionTree.addChild(trappedAction);
//      decisionTree.addChild(goForward);
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
      evaluationTree.tick();
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
      deliverFlag.reset();
   }

   @Override
   public void exit()
   {
      // TODO Auto-generated method stub
      
   }
   
 

}
