package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;

public interface RobotChallengeRobot
{
   public abstract double getHealth();

   public abstract void eatFood(Food01 food);

   public abstract void getHitByPredator(Predator01 predator);

   public abstract Sprite getSprite();

   public abstract void doDynamicsAndUpdateSprite(double dt);

   public abstract Point2D getPosition();
   
   public abstract Vector2D getVelocityVector();

   public abstract Vector2D getHeadingVector();

   public abstract Flag dropFlag();

   public abstract void capturedFlag(Flag flag);

   public abstract void teleportHome();

   public abstract void hitWall();

}
