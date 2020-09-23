package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge04.SimpleRobot04Behavior;

public class RobotChallenge05 
{

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge05", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);

      Robot05Behavior simpleBehavior = new SimpleRobot04Behavior();
      RobotChallengeRules rules = new RobotChallengeRules05(robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), simpleBehavior);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }

}
