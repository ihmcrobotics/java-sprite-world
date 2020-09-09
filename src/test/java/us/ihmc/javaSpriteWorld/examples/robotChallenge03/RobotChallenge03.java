package us.ihmc.javaSpriteWorld.examples.robotChallenge03;

import java.util.Random;

import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRobot;
import us.ihmc.javaSpriteWorld.examples.robotChallenge01.RobotChallengeRules;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.Robot02;
import us.ihmc.javaSpriteWorld.examples.robotChallenge02.RobotChallenge02;

public class RobotChallenge03 extends RobotChallenge02
{

   public RobotChallenge03(String name, RobotChallengeRobot robot, Random random, double xMax, double yMax)
   {
      super(name, robot, random, xMax, yMax);
   }

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;
      
      Robot02 robot = new Robot02(xMax, yMax);

      RobotChallenge03 robotChallenge = new RobotChallenge03("RobotChallenge03", robot, random, xMax, yMax);
      robotChallenge.createSomeFood(10);
      double maximumPredatorSpeed = 1.5;
      robotChallenge.createSomePredators(3, maximumPredatorSpeed);

      Robot03Behavior simpleBehavior = new VectorFieldBehavior03(); // new ExperimentalBehavior03(); // new SimpleRobot03Behavior(); //
      RobotChallengeRules rules = new RobotChallengeRules03(robot, robotChallenge.getFoodList(), robotChallenge.getPredatorList(), simpleBehavior);

      robotChallenge.setRootChallengeRules(rules);
      robotChallenge.runSimulation();
   }

}
