package testRobots;

import suppliedFiles.DifferentialDriveRequest;
import suppliedFiles.LaserEchoesResponse;
import suppliedFiles.LaserPropertiesResponse;
import suppliedFiles.LocalizationResponse;
import suppliedFiles.RobotCommunication;


/**
 * TestRobot interfaces to the (real or virtual) robot over a network
 * connection. It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 ->
 * Lokarria(Robulab) -> Core -> MRDS4
 * 
 * @author Thomas Johansson, dept. of Computing Science, Umeå University, Umeå,
 *         Sweden Mail: thomasj@cs.umu.se
 * 
 *         Updated by Ola Ringdahl 2014-09-10
 */
public class TestRobotLaser
{
   private RobotCommunication robotcomm; // communication drivers

   /**
    * Create a robot connected to host "host" at port "port"
    * 
    * @param host
    *           normally http://127.0.0.1
    * @param port
    *           normally 50000
    */
   public TestRobotLaser(String host, int port)
   {
      robotcomm = new RobotCommunication(host, port);
   }

   /**
    * This simple main program creates a robot, sets up some speed and turning
    * rate and then displays angle and position for 16 seconds.
    * 
    * @param args
    *           not used
    * @throws Exception
    *            not caught
    */
   public static void main(String[] args) throws Exception
   {
      System.out.println("Creating Robot");
      TestRobotLaser robot = new TestRobotLaser("http://127.0.0.1", 50000);

      robot.run();
   }

   private void run() throws Exception
   {
      System.out.println("Creating responses");
      LocalizationResponse lr = new LocalizationResponse();
      LaserEchoesResponse ler = new LaserEchoesResponse();
      LaserPropertiesResponse lpr = new LaserPropertiesResponse();

      System.out.println("Creating request");
      DifferentialDriveRequest dr = new DifferentialDriveRequest();

      // set up the request to move in a circle
      dr.setAngularSpeed(Math.PI * 0.25);
      dr.setLinearSpeed(0.02);

      System.out.println("Start to move robot");
      int rc = robotcomm.putRequest(dr);
      System.out.println("Response code " + rc);

      // Ask for the laser beam angles
      robotcomm.getResponse(lpr);
      double[] angles = getLaserAngles(lpr);

      for (int i = 0; i < 25; i++)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }

         // ask the robot about its position and angle
         robotcomm.getResponse(lr);

         //double angle = getBearingAngle(lr);
         double angle = lr.getHeadingAngle();
         System.out.println("bearing = " + angle * 180 / 3.1415926);

         double[] position = getPosition(lr);
         System.out.println("position = " + position[0] + ", " + position[1]);

         // Ask the robot for laser echoes
         robotcomm.getResponse(ler);
         double[] echoes = ler.getEchoes();

         System.out.println("Object at " + echoes[135] + "m in " + angles[135]
               * 180.0 / Math.PI + " degrees");

      }

      System.out.println("Angle at 0: " + angles[0] * 180.0 / Math.PI
            + " at 45: " + angles[45] * 180.0 / Math.PI + " at 90: "
            + angles[90] * 180.0 / Math.PI + " at 225: " + angles[225] * 180.0
            / Math.PI + "\nAngle at 268: " + angles[268] * 180.0 / Math.PI
            + " at 269: " + angles[269] * 180.0 / Math.PI);

      // set up request to stop the robot
      dr.setLinearSpeed(0);
      dr.setAngularSpeed(0);

      System.out.println("Stop robot");
      rc = robotcomm.putRequest(dr);
      System.out.println("Response code " + rc);

   }

   /**
    * Extract the robot bearing from the response
    * 
    * @param lr
    * @return angle in degrees
    */
   double getBearingAngle(LocalizationResponse lr)
   {
      double e[] = lr.getOrientation(); //e =  {w, x, y, z}

      double angle = 2 * Math.atan2(e[0], e[3]);
      return angle * 180 / Math.PI;
   }

   /**
    * Extract the position
    * 
    * @param lr
    * @return coordinates x, y, z
    */
   double[] getPosition(LocalizationResponse lr)
   {
      return lr.getPosition();
   }

   /**
    * Get corresponding angles to each laser beam
    * 
    * @param lpr
    * @return laser angles in radians
    */
   double[] getLaserAngles(LaserPropertiesResponse lpr)
   {
      int beamCount = (int) ((lpr.getEndAngle() - lpr.getStartAngle()) / lpr
            .getAngleIncrement());
      double[] angles = new double[beamCount];
      double a = lpr.getStartAngle();
      for (int i = 0; i < beamCount; i++)
      {
         angles[i] = a;
         // We get roundoff errors if we use AngleIncrement. Use 1 degree in
         // radians instead
         a += 1 * Math.PI / 180;// lpr.getAngleIncrement();
      }
      return angles;
   }
}
