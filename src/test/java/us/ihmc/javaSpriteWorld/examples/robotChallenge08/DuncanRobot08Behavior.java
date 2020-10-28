package us.ihmc.javaSpriteWorld.examples.robotChallenge08;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.javaSpriteWorld.examples.robotChallenge05.DuncanRobot05Behavior;

import java.util.ArrayList;
import java.util.List;

public class DuncanRobot08Behavior extends DuncanRobot05Behavior
{
   public DuncanRobot08Behavior(boolean runSCS)
   {
      super(runSCS);
   }

   @Override
   protected List<LineSegment2D> getWalls()
   {
      ArrayList<LineSegment2D> walls = new ArrayList<>();
      walls.add(new LineSegment2D(0.0, 0.0, 0.0, getMapSizeY()));
      walls.add(new LineSegment2D(0.0, getMapSizeY(), 10.0, getMapSizeY()));
      walls.add(new LineSegment2D(getMapSizeX(), getMapSizeY(), getMapSizeX(), 0.0));
      walls.add(new LineSegment2D(getMapSizeX(), 0.0, 0.0, 0.0));

      double x0 = 0.0;
      double x1 = 0.35 * getMapSizeX();
      double x2 = 0.6 * getMapSizeX();

      double y0 = 0.0;
      double y1 = 0.3 * getMapSizeY();
      double y2 = 0.45 * getMapSizeY();
      double y3 = 0.65 * getMapSizeY();
      double y4 = 0.8 * getMapSizeY();

      walls.add(new LineSegment2D(x0, y2, x1, y2));
      walls.add(new LineSegment2D(x1, y2, x1, y3));
      walls.add(new LineSegment2D(x1, y4, x1, getMapSizeY()));

      walls.add(new LineSegment2D(x2, y0, x2, y1));
      walls.add(new LineSegment2D(x2, y2, x2, y3));
      walls.add(new LineSegment2D(x2, y3, getMapSizeX(), y3));

      walls.add(new LineSegment2D(x2, y4, x2, getMapSizeY()));

      return walls;
   }

   @Override
   protected double getMapSizeX()
   {
      return 20.0;
   }

   @Override
   protected double getMapSizeY()
   {
      return 20.0;
   }

   @Override
   protected int getNumberOfFlags()
   {
      return 6;
   }

   @Override
   protected int getNumberOfFood()
   {
      return 30;
   }
}
