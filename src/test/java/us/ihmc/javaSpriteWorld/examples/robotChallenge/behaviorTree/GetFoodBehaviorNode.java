package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.bodyToWorld;
import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.doAttractionVectorControl;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

public class GetFoodBehaviorNode implements BehaviorTreeAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private double accelerationGain = 1.0;
   private double turnRateGain = 5.0;
   private double turnRateDamping = -0.5;
   private double attractionStrength = 0.5;
   private double attractionGraduation = 1.5;

   private final double minimumHealthToBeNotHungry = 90.0;
   private double minimumHealthToNotBeDesparateForFarAwayFood = 50.0;
   private double minimumDistanceForFoodToBeConsideredClose = 10.0;

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
      if (notHungry()) return BehaviorTreeNodeStatus.SUCCESS;
      
      
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
      
      if (notHungryEnoughToChaseDownClosestFood(closestDistance))
         return BehaviorTreeNodeStatus.SUCCESS;


      double finalClosestDistance = closestDistance;
      Vector2D attraction = RobotBehaviorTools.fieldVector(sensors.getGlobalPosition(),
                                                           bodyToWorld(sensors, closestFood.getMiddle()), 
                                                           distance -> attractionStrength / Math.pow(finalClosestDistance, attractionGraduation));

      lastVelocity = doAttractionVectorControl(sensors, actuators, attraction, lastVelocity, dt, accelerationGain, turnRateGain, turnRateDamping);

      return BehaviorTreeNodeStatus.RUNNING;
   }

   private boolean notHungryEnoughToChaseDownClosestFood(double closestDistance)
   {
      return (sensors.getHealth() > minimumHealthToNotBeDesparateForFarAwayFood) && (closestDistance > minimumDistanceForFoodToBeConsideredClose);
   }

   private boolean notHungry()
   {
      return sensors.getHealth() > minimumHealthToBeNotHungry;
   }
   
   public void setAttractionStrength(double attractionStrength)
   {
      this.attractionStrength = attractionStrength;
   }

   public void setAttractionGraduation(double attractionGraduation)
   {
      this.attractionGraduation = attractionGraduation;
   }

   public void setAccelerationGain(double accelerationGain)
   {
      this.accelerationGain = accelerationGain;
   }

   public void setTurnRateGain(double turnRateGain)
   {
      this.turnRateGain = turnRateGain;
   }

   public void setTurnRateDamping(double turnRateDamping)
   {
      this.turnRateDamping = turnRateDamping;
   }
}
