package us.ihmc.javaSpriteWorld.examples.robotChallenge02;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallenge01;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRobot;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;

public class RobotChallenge02 extends RobotChallenge01
{

   public RobotChallenge02(String name, RobotChallengeRobot robot, Random random, double xMax, double yMax)
   {
      super(name, robot, random, xMax, yMax);
   }

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge02 robotChallenge = new RobotChallenge02("RobotChallenge02", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);

      Robot02Behavior simpleBehavior = new SimpleRobot02Behavior();
      RobotChallengeRules rules = new RobotChallengeRules02(robot, robotChallenge.getFoodList(), simpleBehavior);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }
}
