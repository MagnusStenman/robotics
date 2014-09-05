
/**
 * Quaternions and some operations on them.
 * The objects are immutable, i.e. once created, they can not be altered.
 * All methods that return a Quaternion create a new one.
 * 
 * @author ThomasJ 12-09-06
 *
 */
public class Quaternion
{
   private static final int W = 0;
   private static final int X = 1;
   private static final int Y = 2;
   private static final int Z = 3;
   
   // Local data, four elements (W, X, Y, Z)
   private double[] data;
   
   /**
    * Construct an immutable quaternion
    * @param w quaternion element data
    * @param x
    * @param y
    * @param z
    */
   public Quaternion(double w, double x, double y, double z)
   {
      data = new double[] {w, x, y, z};
   }
   
   /**
    * Construct a new quaternion with W = 0.0
    * @param x quaternion element data
    * @param y
    * @param z
    */
   public Quaternion(double x, double y, double z)
   {
      this(0.0, x, y, z);
   }
   
   /**
    * Construct a new quaternion from a four-element vector.
    * @param data quaternion element data
    */
   public Quaternion(double[] data)
   {
      this.data = data.clone();
   }
   
   /**
    * Rotate according to the argument q, return a new quaternion
    * @param q Quaternion rotation
    * @return
    */
   public Quaternion rotate(Quaternion q)
   {
      return multiply(q).multiply(conjugate());
   }
   
   /**
    * Returns a new Quaternion which is the conjugate of this
    * @return Conjugate
    */
   public Quaternion conjugate()
   {
      return new Quaternion (new double[] {data[W], -data[X], -data[Y], -data[Z]}); 
   }
   
   /**
    * Return the bearing vector (x, y, z)
    * @return bearing
    */
   public double[] bearing()
   {
      return rotate(new Quaternion(0.0, 1.0, 0.0, 0.0)).getVector();
   }
   
   /**
    * Return the x, y, x components
    * @return
    */
   public double[] getVector()
   {
      return new double[] {data[X], data[Y], data[Z]};
   }
   
   /**
    * Multiply with another quaternion, return a new one
    * @param q
    * @return
    */
   public Quaternion multiply(Quaternion q)
   {
      return new Quaternion(new double[] {
            data[W] * q.data[W] - data[X]*q.data[X] - data[Y]*q.data[Y] - data[Z]* q.data[Z],
            data[W] * q.data[X] + data[X]*q.data[W] + data[Y]*q.data[Z] - data[Z]* q.data[Y],
            data[W] * q.data[Y] - data[X]*q.data[Z] + data[Y]*q.data[W] + data[Z]* q.data[X],
            data[W] * q.data[Z] + data[X]*q.data[Y] - data[Y]*q.data[X] + data[Z]* q.data[W]
      });
   }
   
}
