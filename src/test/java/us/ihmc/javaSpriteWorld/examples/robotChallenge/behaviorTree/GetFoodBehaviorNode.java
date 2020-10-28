package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.*;

public class GetFoodBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;

  
   private double dt = 0.01;
   private double velocity, lastVelocity = 0.0; // PD controller
   private Vector2D headingVector = new Vector2D(0.0, 1.0);
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = new ArrayList<>();

   
   public GetFoodBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {

      Triple<Integer, Point2D, Vector2D> closestFood = null;
      double closestDistance = Double.POSITIVE_INFINITY;
      for (Triple<Integer, Point2D, Vector2D> food : sensors.getLocationOfAllFoodInBodyFrame())
      {
         double distance = sensors.getGlobalPosition().distance(food.getMiddle());
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestFood = food;
         }
      }

      double finalClosestDistance = closestDistance;
      Vector2D attraction = RobotBehaviorTools.fieldVector(sensors.getGlobalPosition(),
                                                           bodyToWorld(sensors, closestFood.getMiddle()), 
                                                           distance -> 0.5 / Math.pow(finalClosestDistance, 1.5));

      headingVector.set(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(sensors.getHeading());
      transform.transform(headingVector);

      double desiredSpeed = attraction.length();
      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attraction.getX(),
                                                                                    attraction.getY());

      actuators.setAcceleration(1.0 * (desiredSpeed - velocity));

      double angularVelocity = (velocity - lastVelocity) / dt;
      double turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
      actuators.setTurnRate(turnRate);
      
      lastVelocity = velocity;

      return BehaviorTreeNodeStatus.RUNNING;
   }
}
