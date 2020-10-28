package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

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
   private double lastVelocity = 0.0; // PD controller

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

      lastVelocity = doAttractionVectorControl(sensors, actuators, attraction, lastVelocity, dt);

      return BehaviorTreeNodeStatus.RUNNING;
   }
}
