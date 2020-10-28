package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.SequenceNode;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;

public class BehaviorTreeBehavior implements Robot05Behavior
{
   private final SequenceNode behaviorTree;
   private final RobotBehaviorActuators actuators = new RobotBehaviorActuators();
   private final RobotBehaviorSensors sensors = new RobotBehaviorSensors();

   private double dt = 0.01;
   private double turnRate, acceleration;
   private double velocity, lastVelocity = 0.0; // PD controller
   private double heading;
   private Vector2D headingVector = new Vector2D(0.0, 1.0);
   private Point2D me = new Point2D(1.5, 1.5);
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = new ArrayList<>();

   public BehaviorTreeBehavior()
   {
      behaviorTree = new SequenceNode();

      AvoidPredatorsBehaviorNode avoidPredators = new AvoidPredatorsBehaviorNode(sensors, actuators);
      GetFoodBehaviorNode getFood = new GetFoodBehaviorNode(sensors, actuators);

      behaviorTree.addChild(avoidPredators);
      behaviorTree.addChild(getFood);
      BehaviorTreeAction eatFood = () ->
      {
         Triple<Integer, Point2D, Vector2D> closestFood = null;
         double closestDistance = Double.POSITIVE_INFINITY;
         for (Triple<Integer, Point2D, Vector2D> food : locationOfAllFood)
         {
            double distance = me.distance(food.getMiddle());
            if (distance < closestDistance)
            {
               closestDistance = distance;
               closestFood = food;
            }
         }

         double finalClosestDistance = closestDistance;
         Vector2D attraction = fieldVector(me, bodyToWorld(closestFood.getMiddle()), distance -> 0.5 / Math.pow(finalClosestDistance, 1.5));

         headingVector.set(0.0, 1.0);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getRotation().appendYawRotation(heading);
         transform.transform(headingVector);

         double desiredSpeed = attraction.length();
         double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                       headingVector.getY(),
                                                                                       attraction.getX(),
                                                                                       attraction.getY());

         acceleration = 1.0 * (desiredSpeed - velocity);

         double angularVelocity = (velocity - lastVelocity) / dt;
         turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
         lastVelocity = velocity;

         return BehaviorTreeNodeStatus.RUNNING;
      };
      
      behaviorTree.addChild(eatFood);
   }

   private Point2D bodyToWorld(Point2DReadOnly pointInBody)
   {
      Point2D pointInWorld = new Point2D(pointInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-heading, 0.0, 0.0);
      pointInWorld.applyInverseTransform(transform);
      pointInWorld.add(me);
      return pointInWorld;
   }

   private Vector2D bodyToWorld(Vector2DReadOnly vectorInBody)
   {
      Vector2D vectorInWorld = new Vector2D(vectorInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-heading, 0.0, 0.0);
      vectorInWorld.applyInverseTransform(transform);
      return vectorInWorld;
   }

   private Vector2D fieldVector(Tuple2DReadOnly from, Tuple2DReadOnly to, Function<Double, Double> magnitude)
   {
      Vector2D vector = new Vector2D(to);
      vector.sub(from);
      double distance = vector.length();
      vector.normalize();
      vector.scale(magnitude.apply(distance));
      return vector;
   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      me.set(x, y);
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
      this.heading = heading;
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
