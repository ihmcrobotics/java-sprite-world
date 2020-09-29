package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotChallengeRules01 implements RobotChallengeRules
{
   private final Robot01 robot;
   private final FoodList01 foodList;
   private final Robot01Behavior robot01Behavior;
   
   public RobotChallengeRules01(Robot01 robot, FoodList01 foodList, Robot01Behavior robot01Behavior)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.robot01Behavior = robot01Behavior;
   }
   
   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      robot01Behavior.senseMousePressed(mousePressedX, mousePressedY); 
   }
   
   @Override
   public void executeRules()
   {
      if (robot01Behavior != null)
      {
         robot01Behavior.senseGlobalLocation(robot.getX(), robot.getY());

         ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
         robot01Behavior.senseFood(locationOfAllFood);

         double[] xyVelocity = robot01Behavior.getXYVelocity();

         robot.setXDot(xyVelocity[0]);
         robot.setYDot(xyVelocity[1]);
      }
   }

   @Override
   public void droppedFlag(int id)
   {
      robot01Behavior.droppedFlag(id);
   }

   @Override
   public void capturedFlag(int id)
   {
      robot01Behavior.pickedUpFlag(id);
   }

   @Override
   public void deliveredFlag(int flagId)
   {
      robot01Behavior.deliveredFlag(flagId); 
   }

   @Override
   public void hitWall()
   {
      robot01Behavior.hitWall();
   }

   
}
