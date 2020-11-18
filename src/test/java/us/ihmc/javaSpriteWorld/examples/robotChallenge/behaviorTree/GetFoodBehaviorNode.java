package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.bodyToWorld;
import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.doAttractionVectorControl;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.utility.UtilityBasedAction;
import us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils;

public class GetFoodBehaviorNode extends UtilityBasedAction
{
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private double accelerationGain = 1.0;
   private double turnRateGain = 5.0;
   private double turnRateDamping = -0.5;
   private double attractionStrength = 10.0;
   private double attractionGraduation = 1.5;

   private double getFoodAttractionVectorConstantLength = 10.0;

   private final double hungerThreshold = 80.0;
   private double desperateHungerThreshold = 50.0;

   private double foodCloseishAndInFrontThreshold = 3.0;
   private double foodCloseishAndBehindThreshold = 0.8;

   private double dt = 0.01;
   private double lastVelocity = 0.0; // PD controller

   private final double[] hungerAction = new double[2];

   public GetFoodBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators)
   {
      this.sensors = sensors;
      this.actuators = actuators;
   }

   @Override
   public double evaluateUtility()
   {
      Point2D closestFoodPositionInBody = sensors.getClosestFoodPositionInBody();
      double headingOfClosestFood = EuclidCoreTools.trimAngleMinusPiToPi(BehaviorUtils.headingFromVector(closestFoodPositionInBody));
      double distanceThreshold = EuclidCoreTools.interpolate(foodCloseishAndInFrontThreshold, foodCloseishAndBehindThreshold, Math.abs(headingOfClosestFood) / Math.PI);

      boolean foodIsCloseish = sensors.getClosestFoodDistance() < distanceThreshold;

      boolean hungry = sensors.getHealth() < hungerThreshold;
      boolean desperatelyHungry = sensors.getHealth() < desperateHungerThreshold;
      double utility = (hungry && foodIsCloseish) || desperatelyHungry ? 1.0 : 0.0;
      return utility;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      Triple<Integer, Point2D, Vector2D> closestFood = null;
      double closestDistance = Double.POSITIVE_INFINITY;
      for (Triple<Integer, Point2D, Vector2D> food : sensors.getLocationOfAllFoodInBodyFrame())
      {
         Point2D foodLocation = bodyToWorld(sensors, food.getMiddle());
         double distance = sensors.getGlobalPosition().distance(foodLocation);
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestFood = food;
         }
      }

      double finalClosestDistance = closestDistance;
      Vector2D attraction = RobotBehaviorTools.fieldVector(sensors.getGlobalPosition(),
                                                           bodyToWorld(sensors, closestFood.getMiddle()),
                                                           distance -> attractionStrength / Math.pow(finalClosestDistance, attractionGraduation));

      attraction.normalize();
      attraction.scale(getFoodAttractionVectorConstantLength);
      lastVelocity = doAttractionVectorControl(sensors, hungerAction, attraction, lastVelocity, dt, accelerationGain, turnRateGain, turnRateDamping);

      actuators.setAcceleration(hungerAction[0]);
      actuators.setTurnRate(hungerAction[1]);
      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
