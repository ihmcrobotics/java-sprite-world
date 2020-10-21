package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotChallengeRules01 implements RobotChallengeRules
{
   private final Robot01 robot;
   private final FoodList01 foodList;
   private final Robot01Behavior robotBehavior;
   
   public RobotChallengeRules01(Robot01 robot, FoodList01 foodList, Robot01Behavior robot01Behavior)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.robotBehavior = robot01Behavior;
   }

   @Override
   public void senseKeyPressed(String keyPressed)
   {
      robotBehavior.senseKeyPressed(keyPressed);
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

         ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
         robotBehavior.senseFood(locationOfAllFood);

         Flag flagHolding = robot.getFlagHolding();
         if (flagHolding == null)
         {
            robotBehavior.senseCarryingFlag(-1);
         }
         else
         {
            robotBehavior.senseCarryingFlag(flagHolding.getId());
         }

         double[] xyVelocity = robotBehavior.getXYVelocity();

         robot.setXDot(xyVelocity[0]);
         robot.setYDot(xyVelocity[1]);
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

   @Override
   public void reportScoreHealthTime(double score, double health, double time)
   {
      robotBehavior.senseScoreHealthTime(score, health, time);
   }

   @Override
   public void reset()
   {
      robotBehavior.reset();
   }

}
