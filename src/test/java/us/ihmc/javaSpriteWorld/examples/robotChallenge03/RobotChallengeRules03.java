package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.FoodList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.PredatorList01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;

public class RobotChallengeRules03 implements RobotChallengeRules
{
   private final Robot02 robot;
   private final FoodList01 foodList;
   private final PredatorList01 predatorList;
   private final Robot03Behavior robotBehavior;
   
   public RobotChallengeRules03(Robot02 robot, FoodList01 foodList, PredatorList01 predatorList, Robot03Behavior robotBehavior)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.predatorList = predatorList;
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
         

         ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood = foodList.getLocationAndVelocityOfAllFood();
         robotBehavior.senseFood(locationOfAllFood);
         
         ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators = predatorList.getLocationAndVelocityOfAllPredators();
         robotBehavior.sensePredators(locationOfAllPredators);

         double[] accelerationAndTurnRate = robotBehavior.getAccelerationAndTurnRate();

         robot.setAcceleration(accelerationAndTurnRate[0]);
         robot.setTurnRate(accelerationAndTurnRate[1]);
      }
   }

   
}
