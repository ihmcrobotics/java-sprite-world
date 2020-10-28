package us.ihmc.javaSpriteWorld.examples.duncan;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.javaSpriteWorld.examples.duncan.DuncanRobot06Behavior;

import java.util.Arrays;
import java.util.List;

public class DuncanRobot07Behavior extends DuncanRobot06Behavior
{
   @Override
   protected List<LineSegment2D> getWalls()
   {
      LineSegment2D left = new LineSegment2D(0.0, 0.0, 0.0, 10.0);
      LineSegment2D top = new LineSegment2D(0.0, 10.0, 10.0, 10.0);
      LineSegment2D right = new LineSegment2D(10.0, 10.0, 10.0, 0.0);
      LineSegment2D bottom = new LineSegment2D(10.0, 0.0, 0.0, 0.0);
      LineSegment2D wall = new LineSegment2D(2.0/3.0*10.0, 1.0/3.0 * 10.0, 2.0/3.0*10.0, 2.0/3.0*10.0);
      return Arrays.asList(left, right, bottom, top, wall);
   }
}
