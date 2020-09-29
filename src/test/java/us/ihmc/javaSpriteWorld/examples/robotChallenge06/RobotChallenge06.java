package us.ihmc.javaSpriteWorld.examples.robotChallenge06;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.DuncanRobot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.Robot05Behavior;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.SimpleRobot05Behavior;

public class RobotChallenge06
{
   public static final String PLAYER = System.getProperty("player", "Simple");

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge01 robotChallenge = new RobotChallenge01("RobotChallenge06", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);
      robotChallenge.createSomeFlags(5);

      Robot06Behavior simpleBehavior;
      if (PLAYER.equals("Duncan"))
         simpleBehavior = new DuncanRobot05Behavior();
      else if (PLAYER.equals("Stephen"))
         simpleBehavior = new SimpleRobot05Behavior(); // TODO: Stephen adds behavior here
      else
         simpleBehavior = new SimpleRobot05Behavior();
      RobotChallengeRules rules = new RobotChallengeRules06(robotChallenge, robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), robotChallenge.getFlagList(), simpleBehavior);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }

}
