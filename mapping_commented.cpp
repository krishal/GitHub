/*
 *
 *Lab 2 Occ. Grid Code
 *Group 17
 *Fall 2015
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gazebo_msgs/ModelStates.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x, ips_y, ips_yaw;
double dif_x, dif_y, dif_yaw;
short sgn(int x) { return x >= 0 ? 1 : -1; }
bool NEW_VALUES = false;
bool NEW_POS = false;

const int UNITS_PER_METER = 50;
float angle_min = 0;
float angle_max = 0;
float angle_inc = 0;
int range_min = 0;
int range_max = 5*UNITS_PER_METER;
int maxNumRanges = 360;
int numRanges = 0;
std::vector<float> ranges(maxNumRanges);

const int M = 10*UNITS_PER_METER;
const int N = 10*UNITS_PER_METER;
float emptyWeigth = 0.2;
float occupyWeight = 0.8;

//Callback function for the Position topic (SIMULATION)
/*void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) 
        if(msg.name[i] == "mobile_base") 
            break;
    dif_x = abs(ips_x - msg.pose[i].position.x);
    dif_y = abs(ips_y - msg.pose[i].position.y);
    dif_yaw = abs(ips_yaw - tf::getYaw(msg.pose[i].orientation));
    if(dif_yaw > 0.01 || dif_x > 0.01 || dif_y > 0.01)
    {
        NEW_POS = true;

        ips_x = msg.pose[i].position.x ;
        ips_y = msg.pose[i].position.y ;
        ips_yaw = tf::getYaw(msg.pose[i].orientation);
    }
}*/

//Callback function for the Position topic (LIVE)
void pose_callbackLive(const geometry_msgs::PoseWithCovarianceStamped& msg){
    //Gets the difference between the current pos and the previous to check for error
	dif_x = abs(ips_x - msg.pose.pose.position.x);
    dif_y = abs(ips_y - msg.pose.pose.position.y);
    //Flips the yaw to match the live positioning system
    float temp_yaw = tf::getYaw(msg.pose.pose.orientation);
    if(temp_yaw>0){
        temp_yaw = temp_yaw*-1+M_PI;
    } 
    else{
        temp_yaw = temp_yaw*-1-M_PI;
    }
    //Checks the difference in Yaw
    dif_yaw = abs(ips_yaw - temp_yaw);

    //Only updates position if it is above the error threshold
    if(dif_yaw > 0.01 || dif_x > 0.01 || dif_y > 0.01){
    	//Indicates that a new position has been recieved
        NEW_POS = true;
        //Sets the new position
		ips_x = 2.17*(msg.pose.pose.position.x); // Robot X position
        ips_y = 2.17*(msg.pose.pose.position.y); // Robot Y position
        ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
        //Flips the yaw
        if(ips_yaw>0)
            ips_yaw = ips_yaw*-1+M_PI;
        else
            ips_yaw = ips_yaw*-1-M_PI;
    }
}


void las_callback(const sensor_msgs::LaserScan& msg)
{
	//Sets the values for the angle from the specific laser
    angle_min = msg.angle_min;
    angle_max = msg.angle_max;
    angle_inc = msg.angle_increment;

    //Sets the max and min values for the Laser
    range_min = (int)(floor(msg.range_min*UNITS_PER_METER));
    range_max = (int)(floor(msg.range_max*UNITS_PER_METER));

    //Checks if the sensor data is new
    for(int i =0; i< msg.ranges.size() && NEW_VALUES==false; i++){
        if(abs(ranges[i] - msg.ranges[i]) > 0.1 && msg.ranges[i]>1 && msg.ranges[i]<4){
            NEW_VALUES = true;
        }
    }
    //Updates the values if the range is new
    if(NEW_VALUES){
    	//Updates the range values for the laser
        ranges = msg.ranges;
        numRanges = msg.ranges.size();
    }
}


//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
//void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {
template <typename Derived>
int bresenham(int x0, int y0, int x1, int y1, ArrayBase<Derived>& invmod) 
{
    x1 = x1 + M/2;
    y1 = y1 + N/2;
    x0 = x0 + M/2;
    y0 = y0 + N/2;
    int xr = x0;
    int yr = y0;
    float r = 0;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    int invmodSize = 1;
    
    //Check if change in y is greater than change in x
    const bool s = abs(dy) > abs(dx);

    //if dy is greater then swap dx and dy
    if (s){
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    //
    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    invmod = ArrayXXf::Zero(range_max, 3);

    if(invmod.rows()<range_max||invmod.cols()<3){
        ROS_INFO("OOPS row=%ld, col=%ld",invmod.rows(),invmod.cols());
        return 0;
    }
    invmod(0,0) = xr;
    invmod(0,1) = yr;
    invmod(0,2) = emptyWeigth;

    while (xr != x1 || yr != y1){
        if (s) 
        	yr+=sgn(dy2); 
        else 
        	xr+=sgn(dx2);

        if (d < 0)
        	d += inc1;
        else{
            d += inc2;
            if (s) xr+=sgn(dx2); else yr+=sgn(dy2);
        }

        r = sqrt((xr-x0)*(xr-x0) + (yr-y0)*(yr-y0));
        //Add point to vector
        invmod(invmodSize,0) = xr;
        invmod(invmodSize,1) = yr;
        invmod(invmodSize,2) = emptyWeigth;

        invmodSize++;
    }
    return invmodSize;
}

int main(int argc, char **argv)
{
	//Sets up the arrays for the occupancy grid
    Array<float,Dynamic,Dynamic,ColMajor> m = ArrayXXf::Constant(M,N,.5);
    ArrayXXf L0 = (m/(1-m)).log();
    ArrayXXf L = L0;
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscriber for the position and the laser
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callbackLive);
    //ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber kin_sub = n.subscribe("/scan", 1, las_callback);

    //Publisher for the map
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    
    //Sets up the variables needed to define the information published for the occupancy grid
    nav_msgs::OccupancyGrid occupyGrid;
    std_msgs::Header head_map;
    nav_msgs::MapMetaData meta;
    geometry_msgs::Pose mapPos;
    std_msgs::Header head_pos;

    //Set the loop rate
    ros::Rate loop_rate(10);    //10Hz update rate
    
    //Sets up variables for the Inverse Scanner Bresenham
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    float theta = 0;
    ArrayXXf invmod = ArrayXXf::Zero(range_max,3);
    int invmodSize = 0;

    int seq = 0;
    while (ros::ok()){
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
        //Checks if new data has been recieved
        if(NEW_VALUES && NEW_POS){
        	//Loops through the array for 
            for(int i = 0; i<numRanges; i++){
                if(isnan(ranges[i])==0){
                    invmod = ArrayXXf::Zero(range_max,3);
                    //inversscannerbres
                    theta = ips_yaw+angle_min+angle_inc*i;
                    x1 = max(-M/2,min(M/2 - 1,(int)(round(ips_x*UNITS_PER_METER))));
                    y1 = max(-N/2,min(N/2 - 1,(int)(round(ips_y*UNITS_PER_METER))));
                    x2 = max(-M/2,min(M/2 - 1,(int)(round(UNITS_PER_METER*(ips_x + ranges[i]*cos(theta))))));
                    y2 = max(-N/2,min(N/2 - 1,(int)(round(UNITS_PER_METER*(ips_y + ranges[i]*sin(theta))))));
                    invmodSize = bresenham(x1,y1,x2,y2,invmod);
                    if(invmodSize>0){
                        invmod(invmodSize-1,2) = occupyWeight;
                    }
                    for(int j =0; j<invmodSize; j++){
                        L(invmod(j,0),invmod(j,1)) = L(invmod(j,0),invmod(j,1)) 
                        + log(invmod(j,2)/(1-invmod(j,2))) - L0(invmod(j,0),invmod(j,1));
                    }
                }

            }
            // Calculate probabilities
            m = L.exp()/(1+L.exp());

            //Sets the header variables for the position header
	        head_pos.seq = seq;
	        head_pos.stamp = ros::Time::now(); //Gets the current time
	        head_pos.frame_id = "/base_link";

	        //Sets the variables for the header meta section
	        head_map.seq = seq;
	        head_map.stamp = head_pos.stamp;
	        head_map.frame_id = "/map";
	        //Increases the sequence number for the next iteration
	        seq++;

	        //Sets the postion and orientation of the map
	        mapPos.position.x = M/-2/UNITS_PER_METER;
	        mapPos.position.y = N/-2/UNITS_PER_METER;
	        mapPos.orientation = tf::createQuaternionMsgFromYaw(0);
	        //Sets the dimensions of the map
	        meta.map_load_time = head_map.stamp;
	        meta.resolution = 1.0/UNITS_PER_METER;
	        meta.width = N;
	        meta.height = M;
	        meta.origin = mapPos;
	        //Sets the variables for the occupancy grid
	        occupyGrid.header = head_map;
	        occupyGrid.info = meta;

	        //Clears the old data from the occupancy grid for the new information
	        occupyGrid.data.clear();
	        for(int k = 0; k < m.size(); k++){
	        	//Pushs new values into the occupancy grid
	            occupyGrid.data.push_back((int8_t)(*(m.data() + k)*100));
	        }
	        //Publishes the grid
        	map_publisher.publish(occupyGrid);

        	//Sets the bools to false 
	        NEW_VALUES = false;
	        NEW_POS = false;
        }
    }
    return 0;
}