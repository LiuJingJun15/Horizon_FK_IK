#include "ros/ros.h"
#include "std_msgs/String.h"
#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61
#include "ikfast61_horizon_arm.cpp"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#define IKREAL_TYPE IkReal // for IKFast 56,61
using namespace std;

int main(int argc, char **argv)
{
  unsigned int num_of_joints = GetNumJoints();
  IkReal eerot_FK[9],eetrans_Fk[3]; // the rotation and transformation matrix for forward kinematics
  IkReal eerot_IK[9],eetrans_IK[3]; // the rotation and transformation matrix for inverse kinematics
  IkSolutionList<IkReal> solutions;
  std::vector<IkReal> vfree(0);
  IKREAL_TYPE joints[num_of_joints];

  /**
  TODO
  another listener to subscribe motor angles and user input
  */
  for (unsigned int i=0; i<num_of_joints; i++)
        {
            joints[i] = 0; // TODO, change this line and assign joint angles
        }

  ComputeFk(joints, eetrans_Fk, eerot_FK); // assign current transformation and rotation

  /**
  the euler angles of FK, used in combining with user input
  */
  float yaw_FK;
  float pitch_FK;
  float roll_FK;
  if ( eerot_FK[5] > 0.998 || eerot_FK[5] < -0.998 ) { // singularity
      yaw_FK = IKatan2( -eerot_FK[6], eerot_FK[0] );
      pitch_FK = 0;
  } else {
      yaw_FK = IKatan2( eerot_FK[2], eerot_FK[8] );
      pitch_FK = IKatan2( eerot_FK[3], eerot_FK[4] );
  }
  roll_FK = IKasin( eerot_FK[5] );


  /**
  add eetrans_FK and eerot_FK with user input

  eetrans_IK[0] = eetrans_FK[0] + userin[0];
  eetrans_IK[0] = eetrans_FK[1] + userin[1];
  eetrans_IK[0] = eetrans_FK[2] + userin[2];
  yaw_IK = yaw_FK + userin[3];
  pitch_IK = pitch_FK + userin[4];
  roll_IK = roll_FK + userin[5];
  
  qx, qy, qz, qw is transformed from three IK euler angles
  TODO: transform euler angles to quaternions.
  */

  /** 
  The following is the desired configuration, should be replacecd
  */
  eetrans_IK[0] = 0.2;
  eetrans_IK[1] = 0.2;
  eetrans_IK[2] = 0.2;
  double qw = 0.2;
  double qx = 0.2;
  double qy = 0.2;
  double qz = 0.2;
  /**
  The following is 
  */
  const double size = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
  qw *= size;
  qx *= size;
  qy *= size;
  qz *= size;
  eerot_IK[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot_IK[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot_IK[2] = 2.0f*qx*qz + 2.0f*qy*qw;
  eerot_IK[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot_IK[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot_IK[5] = 2.0f*qy*qz - 2.0f*qx*qw;
  eerot_IK[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot_IK[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot_IK[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  bool bSuccess = ComputeIk(eetrans_IK, eerot_IK, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

  std::vector<vector<IKREAL_TYPE> > SolvaluesArray(num_of_solutions); // a 2-D vector storing all solution
  std::vector<IKREAL_TYPE> bestSolvalues(6); // this is the best solution we find
  std::vector<IKREAL_TYPE> solvalues(6);
    for(std::size_t i = 0; i < num_of_solutions; ++i) {
        const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
        int this_sol_free_params = (int)sol.GetFree().size(); 
        printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
        std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");

        SolvaluesArray[i] = solvalues;
        }

  /**
  TODO 
  find out the best solution
  bestSolvalues = findBestSolution(SolvaluesArray)
  */

  bestSolvalues = solvalues; // for now the best solution is the last solution
  ros::init(argc, argv, "fk_ik_talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("fk_ik_chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  
  while (ros::ok())
  {
    /**
     * This are the six best solutions, will be published splited with ','
     */
    IKREAL_TYPE a = bestSolvalues[0];
    IKREAL_TYPE b = bestSolvalues[1];
    IKREAL_TYPE c = bestSolvalues[2];
    IKREAL_TYPE d = bestSolvalues[3];
    IKREAL_TYPE e = bestSolvalues[4];
    IKREAL_TYPE f = bestSolvalues[5];
    std_msgs::String msg;

    std::stringstream ss;
    ss << a <<',' << b << ',' << c << ',' << d << ',' << e << ',' << f << ',' << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}