package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;

public class RobotChallengeRules02 implements RobotChallengeRules
{
   private final Robot02 robot;
   private final FoodList01 foodList;
   private final Robot02Behavior robotBehavior;
   
   public RobotChallengeRules02(Robot02 robot, FoodList01 foodList, Robot02Behavior robotBehavior)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.robotBehavior = robotBehavior;
   }
  

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      robotBehavior.senseMousePressed(mousePressedX, mousePressedY); 
   }
   
   @Override
   public void executeRules()
   {
      if (robotBehavior != null)
      {
         robotBehavior.senseGlobalLocation(robot.getX(), robot.getY());
         robotBehavior.senseHeading(robot.getHeading());
         robotBehavior.senseVelocity(robot.getVelocity());

         ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
         robotBehavior.senseFood(locationOfAllFood);

         double[] accelerationAndTurnRate = robotBehavior.getAccelerationAndTurnRate();

         robot.setAcceleration(accelerationAndTurnRate[0]);
         robot.setTurnRate(accelerationAndTurnRate[1]);
      }
   }

   @Override
   public void droppedFlag(int id)
   {
      robotBehavior.senseDroppedFlag(id);
   }

   @Override
   public void capturedFlag(int id)
   {
      robotBehavior.sensePickedUpFlag(id);
   }
   
   @Override
   public void deliveredFlag(int flagId)
   {
      robotBehavior.senseDeliveredFlag(flagId); 
   }

   @Override
   public void hitWall()
   {
      robotBehavior.senseHitWall();      
   }
}
