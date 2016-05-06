package us.ihmc.javaSpriteWorld.geometry;

public class Vector
{
   private double x, y;

   public Vector(double x, double y)
   {
      this.x = x;
      this.y = y;
   }
   
   public Vector()
   {
      this(0.0, 0.0);
   }

   public double getX()
   {
      return x;
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public double getY()
   {
      return y;
   }

   public void setY(double y)
   {
      this.y = y;
   }

   public void set(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public void normalize()
   {
      double length = Math.sqrt(x*x + y*y);
      x = x / length;
      y = y / length;
   }
   
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }
   
   
}
