package us.ihmc.javaSpriteWorld.examples.robotChallenge04;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;

public class RobotChallenge04 
{

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge04", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(1, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);

      Robot04Behavior simpleBehavior = new DuncanRobot04Behavior();
      RobotChallengeRules rules = new RobotChallengeRules04(robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), simpleBehavior);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }

}