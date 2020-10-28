package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.SequenceNode;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;

public class BehaviorTreeBehavior implements Robot05Behavior
{
   private final SequenceNode behaviorTree;
   private final RobotBehaviorActuators actuators = new RobotBehaviorActuators();
   private final RobotBehaviorSensors sensors = new RobotBehaviorSensors();
   
   public BehaviorTreeBehavior()
   {
      behaviorTree = new SequenceNode();
      
      GetFoodBehaviorNode getFood = new GetFoodBehaviorNode(sensors, actuators);
      behaviorTree.addChild(getFood);
   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseNoiseFreeHeadingForTestingOnly(double heading)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseVelocity(double velocity)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseHeading(double heading)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      sensors.setLocationOfAllFood(locationOfAllFood);
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseCarryingFlag(int flagId)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      behaviorTree.tick();
      return new double[] {actuators.getAcceleration(), actuators.getTurnRate()};
   }

   @Override
   public boolean getDropFlag()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double elapsedTime)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void reset()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void exit()
   {
      // TODO Auto-generated method stub
      
   }
   
 

}
