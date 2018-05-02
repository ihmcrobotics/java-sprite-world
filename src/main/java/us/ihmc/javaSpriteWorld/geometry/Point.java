package us.ihmc.javaSpriteWorld.geometry;

public class Point
{
   private double x, y;

   public Point()
   {
      this(0.0, 0.0);
   }

   public Point(Point point)
   {
      this(point.x, point.y);
   }

   public Point(double x, double y)
   {
      this.x = x;
      this.y = y;
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

   public void transform(boolean reflectX, boolean reflectY, double xTranslation, double yTranslation, double rotation)
   {
      double reflectedX = x;
      if (reflectX) reflectedX = -x;
      
      double reflectedY = y;
      if (reflectY) reflectedY = -y;
      
      double rotatedX = reflectedX * Math.cos(rotation) - reflectedY * Math.sin(rotation);
      double rotatedY = reflectedX * Math.sin(rotation) + reflectedY * Math.cos(rotation);

      double translatedX = rotatedX + xTranslation;
      double translatedY = rotatedY + yTranslation;

      this.x = translatedX;
      this.y = translatedY;
   }

   public void set(Point point)
   {
      this.x = point.x;
      this.y = point.y;
   }

   public void set(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public void setToNaN()
   {
      this.x = Double.NaN;
      this.y = Double.NaN;
   }
   
   public boolean isNaN()
   {
      if (Double.isNaN(this.x)) return true;
      if (Double.isNaN(this.y)) return true;
      
      return false;
   }
   
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }

}
