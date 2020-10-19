package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.stephen.StephenRobotBehavior;

public class RobotChallenge05 
{
   public static void main(String[] args)
   {
      String player = System.getProperty("player", "Stephen");
//      String player = System.getProperty("player", "Simple");
      RobotChallenge01 robotChallenge = createRobotChallenge(player);
//      robotChallenge.runSimulation();

      robotChallenge.runATrial(30, 20.0, 60.0);
   }

   private static RobotChallenge01 createRobotChallenge(String player)
   { 
      double xMax = 10.0;
      double yMax = 10.0;

      Random random = new Random();

      Robot05Behavior simpleBehavior = createBehavior(player, xMax, yMax);
      Robot02 robot = new Robot02(xMax, yMax);
      RobotChallenge01 robotChallenge = createRobotChallenge(xMax, yMax, random, robot);

      RobotChallengeRules05 rules = new RobotChallengeRules05(robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), simpleBehavior);      
      robotChallenge.setRobotChallengeRules(rules);
      
      if (player.equals("Simple"))
      {
         rules.setTesting(true);
      }
      return robotChallenge;
   }

   private static RobotChallenge01 createRobotChallenge(double xMax, double yMax, Random random, Robot02 robot)
   {
      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge05", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);
      return robotChallenge;
   }

   private static Robot05Behavior createBehavior(String player, double xMax, double yMax)
   {
      Robot05Behavior simpleBehavior;
      if (player.equals("Duncan"))
         simpleBehavior = new DuncanRobot05Behavior();
      else if (player.equals("Stephen"))
         simpleBehavior = new StephenRobotBehavior();
      else
         simpleBehavior = new SimpleRobot05Behavior(xMax, yMax);
      return simpleBehavior;
   }

}
