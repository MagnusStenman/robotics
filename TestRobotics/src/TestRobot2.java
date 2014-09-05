/**
 * TestRobot interfaces to the (real or virtual) robot over a network connection.
 * It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 -> Lokarria(Robulab) -> Core -> MRDS4
 * 
 * @author thomasj
 */
public class TestRobot2
{
   private RobotCommunication robotcomm;  // communication drivers

   /**
    * Create a robot connected to host "host" at port "port"
    * @param host normally http://127.0.0.1
    * @param port normally 50000
    */
   public TestRobot2(String host, int port)
   {
      robotcomm = new RobotCommunication(host, port);
   }

   /**
    * This simple main program creates a robot, sets up some speed and turning rate and
    * then displays angle and position for 16 seconds.
    * @param args         not used
    * @throws Exception   not caught
    */
   public static void main(String[] args) throws Exception
   {     
      System.out.println("Creating Robot");
      TestRobot2 robot = new TestRobot2("http://127.0.0.1", 50000);

      robot.run();
   }


   private void run() throws Exception
   {
      System.out.println("Creating response");
      LocalizationResponse lr = new LocalizationResponse();

      System.out.println("Creating request");
      DifferentialDriveRequest dr = new DifferentialDriveRequest();

      // set up the request to move in a circle
      dr.setAngularSpeed(Math.PI * 0.25);
      dr.setLinearSpeed(1.0);

      System.out.println("Start to move robot");
      int rc = robotcomm.putRequest(dr);
      System.out.println("Response code " + rc);

      for (int i = 0; i < 16; i++)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex) {}

         // ask the robot about its position and angle
         robotcomm.getResponse(lr);

         double angle = getBearingAngle(lr);
         System.out.println("bearing = " + angle);

         double [] position = getPosition(lr);
         System.out.println("position = " + position[0] + ", " + position[1]); 
      }

      // set up request to stop the robot
      dr.setLinearSpeed(0);
      dr.setAngularSpeed(0);

      System.out.println("Stop robot");
      rc = robotcomm.putRequest(dr);
      System.out.println("Response code " + rc);

   }

   /**
    * Extract the robot bearing from the response
    * @param lr
    * @return angle in degrees
    */
   double getBearingAngle(LocalizationResponse lr)
   {
      double e[] = lr.getOrientation();

      double angle = 2 * Math.atan2(e[3], e[0]);
      return angle * 180 / Math.PI;
   }

   /**
    * Extract the position
    * @param lr
    * @return coordinates
    */
   double[] getPosition(LocalizationResponse lr)
   {
      return lr.getPosition();
   }


}

