#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>


#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//使用Action通信所需要的头文件

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 
nav_msgs::OccupancyGrid Map;

int check_map=0;

void change_map();
void Start_searching_move(int goal[][3], int nums);

void RoutePlan(nav_msgs::OccupancyGrid myMap)
{
    int w = myMap.info.width;
    int h = myMap.info.height;
    float destor =myMap.info.resolution;

    ROS_INFO("hight is %d width is %d resolution is %.5f",h,w,destor);
    ROS_INFO("start");

    
     if(Map.info.height!=0){
        }
    else
    {
        check_map=1;
    }

    if(check_map==1)
    {
        Map=myMap;
        ROS_INFO("saved map");
        int w = Map.info.width;
        int h = Map.info.height;
        ROS_INFO("map_hight is %d map_width is %d",h,w);
        ros::NodeHandle nh_pub;
        ros::Publisher pub = nh_pub.advertise<nav_msgs::OccupancyGrid> ("mappublish", 1);
        ros::Rate rate(1);
        change_map();
         while(1){
                    pub.publish(Map);
                    rate.sleep();       

         }

    }

}
void change_map(){
// load map to 一维数组 size=384*384
    int p[Map.info.width*Map.info.height] = {-1};   // [0,100]
    for(int i=0; i<Map.info.width*Map.info.height;i++){
        p[i] = Map.data[i];
        if(p[i]!=0)
        {
        p[i]=100; // 去除未知区域
        }
        if(p[i]!=100){
            p[i]=0;
        }
    }

    int map_test_goal[Map.info.height][Map.info.width]= {-1};

    // change into matrix
    int map_p[Map.info.height][Map.info.width]={-1};
    for(int i=0; i<Map.info.width*Map.info.height;i++){
        map_p[Map.info.width-i/Map.info.width][Map.info.height-i%Map.info.height]= p[i];
        map_test_goal[Map.info.width-i/Map.info.width][Map.info.height-i%Map.info.height]=Map.data[i];

    }

    //generate the new edge

    int map_new[Map.info.height][Map.info.width]= {-1};
    for(int i=0; i<Map.info.width;i++){
        for(int j=0;j<Map.info.height;j++){
            map_new[i][j]=map_p[i][j];

        }
    }

    int r=6; //distance between route and wall
    vector<vector<int>> goals_all; // 栅格坐标
    // get route
    for(int i=0; i<Map.info.width;i++){
         for(int j=0;j<Map.info.height;j++){
             if(map_p[i][j]!=100){
                
                if(map_p[i][j+r]==100){
                    int check_dis=0;
                    for(int r_i=0;r_i<r;r_i++){
                        for(int r_j=0;r_j<r;r_j++){
                            if((map_p[i-r_i][j-r_j]==100)||(map_p[i+r_i][j-r_j]==100)||(map_p[i-r_i][j+r_j]==100)||(map_p[i+r_i][j+r_j]==100)){
                                check_dis=1;
                                break;
                            }
                        }
                    }
                    if(check_dis==0)
                    {
                        map_new[i][j]=100;
                        vector<int> cord;
                        cord.push_back(i);
                        cord.push_back(j);
                        cord.push_back(3);
                        goals_all.push_back(cord);
                    }
                }
                if(map_p[i][j-r]==100){
                    int check_dis=0;
                    for(int r_i=0;r_i<r;r_i++){
                        for(int r_j=0;r_j<r;r_j++){
                            if((map_p[i-r_i][j-r_j]==100)||(map_p[i+r_i][j-r_j]==100)||(map_p[i-r_i][j+r_j]==100)||(map_p[i+r_i][j+r_j]==100)){
                                check_dis=1;
                                break;
                            }
                        }
                    }
                    if(check_dis==0)
                    {
                        map_new[i][j]=100;                       
                        vector<int> cord;
                        cord.push_back(i);
                        cord.push_back(j);
                        cord.push_back(2);
                        goals_all.push_back(cord);
                    }
                }
                if(map_p[i+r][j]==100){
                    int check_dis=0;
                    for(int r_i=0;r_i<r;r_i++){
                        for(int r_j=0;r_j<r;r_j++){
                            if((map_p[i-r_i][j-r_j]==100)||(map_p[i+r_i][j-r_j]==100)||(map_p[i-r_i][j+r_j]==100)||(map_p[i+r_i][j+r_j]==100)){
                                check_dis=1;
                                break;
                            }
                        }
                    }
                    if(check_dis==0)
                    {
                        map_new[i][j]=100;
                        vector<int> cord;
                        cord.push_back(i);
                        cord.push_back(j);
                        cord.push_back(1);
                        goals_all.push_back(cord);
                    }
                }
                if(map_p[i-r][j]==100){
                    int check_dis=0;
                    for(int r_i=0;r_i<r;r_i++){
                        for(int r_j=0;r_j<r;r_j++){
                            if((map_p[i-r_i][j-r_j]==100)||(map_p[i+r_i][j-r_j]==100)||(map_p[i-r_i][j+r_j]==100)||(map_p[i+r_i][j+r_j]==100)){
                                check_dis=1;
                                break;
                            }
                        }
                    }
                    if(check_dis==0)
                    {
                        map_new[i][j]=100;
                        vector<int> cord;
                        cord.push_back(i);
                        cord.push_back(j);
                        cord.push_back(4);
                        goals_all.push_back(cord);
                        
                    }
                }
             }
        }
    }
    int goal_dense=1;
    int size_goals= goals_all.size();
    int goal_nums=0;
    
    int map_goals[Map.info.height][Map.info.width]= {-1};
    for(int i=0; i<Map.info.width;i++){
        for(int j=0;j<Map.info.height;j++){
            map_goals[i][j]=map_test_goal[i][j];
        }
    }


    for(int i=0; i<size_goals;i=i+goal_dense)
    {
        vector<int> cord;
        cord= goals_all[i];
        map_goals[cord[0]][cord[1]]=100;
        goal_nums++;
    }

    int goals_final_list[goal_nums][3];
    int goals_final_index =0;

    for(int i=0; i<size_goals;i=i+goal_dense)
    {
        vector<int> cord;
        cord= goals_all[i];
        goals_final_list[goals_final_index][0]=cord[0];
        goals_final_list[goals_final_index][1]=cord[1];
        goals_final_list[goals_final_index][2]=cord[2];
        goals_final_index++;
    }
    
    // for(int i=0; i<10;i++)
    // {
    //     map_goals[192+i][192]=100;
    // }
    // for(int i=0; i<20;i++)
    // {
    //     map_goals[192][192+i]=100;
    // }
    // for(int i=0; i<20;i=i+2)
    // {
    //     map_goals[192][192-i]=100;
    // }
    // for(int i=0; i<20;i=i+2)
    // {
    //     map_goals[192-i][192]=100;
    // }

    int test_map[Map.info.width*Map.info.height] = {-1};  
    int index=0;
    for(int i=0; i<Map.info.width;i++){
        for(int j=0;j<Map.info.height;j++){
            test_map[index] = map_goals[Map.info.width-i][Map.info.height-j];
            index++;
        }
    }
    std::vector<signed char> a(test_map,test_map+Map.info.width*Map.info.height);
    Map.data = a;

    Start_searching_move(goals_final_list, goal_nums);

}

void Start_searching_move(int goals[][3], int nums){

    //tell the action client that we want to spin a thread by default
    //don't need ros::spin() 创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
	MoveBaseClient ac("move_base", true);
	 
	//wait for the action server to come up1000
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	//自己初始化一个坐标吧
	geometry_msgs::PoseStamped send_Pose;
	move_base_msgs::MoveBaseGoal goal;
	
    //start place
	send_Pose.pose.position.x = 1;
	send_Pose.pose.position.y = 1;
	send_Pose.pose.position.z = 0;
	send_Pose.pose.orientation.x = 0;
	send_Pose.pose.orientation.y = 0;
	send_Pose.pose.orientation.z = -1;
	send_Pose.pose.orientation.w = 0;
    send_Pose.header.frame_id= "odom";
	goal.target_pose = send_Pose;//此处记得要定义MoveBaseGoal类的goal
	goal.target_pose.header.frame_id = "odom";
	ac.sendGoal(goal);
	
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Ready to go!");
		//TODO: 成功到达目的地，此处发挥想象做点啥
	}
	else{
		ROS_INFO("Failed to reach the goal!");  
	}   
    
    int goal_sort[nums][3];
    goal_sort[0][0]=goals[0][0];
    goal_sort[0][1]=goals[0][1];
    goal_sort[0][2]=goals[0][2];

    for(int i=0; i<nums; i++){

        double dis = 0, dis_min=100000000000;
        int index_closest =0;

        for( int j=0; j<nums; j++){
            dis= ((goal_sort[i][0]-goals[j][0])*(goal_sort[i][0]-goals[j][0])+(goal_sort[i][1]-goals[j][1])*(goal_sort[i][1]-goals[j][1]));
            if(dis!=0 && dis<dis_min){
                dis_min=dis;
                index_closest=j;
            }
        }
        if(i+1<nums){
            goal_sort[i+1][0]=goals[index_closest][0];
            goal_sort[i+1][1]=goals[index_closest][1];
            goal_sort[i+1][2]=goals[index_closest][2];
            goals[index_closest][0]=0;
            goals[index_closest][1]=0;
            goals[index_closest][2]=0;

        }

    }

    int map_size_grid=384;
    double map_resolution=0.05;

    
    for(int i=0; i<nums;i=i+5){

        double goals_x,goals_y;
        goals_x=-(double)(goal_sort[i][0]-192+8)*map_resolution;
        goals_y=-(double)(goal_sort[i][1]-192+8)*map_resolution;

        ROS_INFO("goals num: %d, x: %.3f, y: %.3f,  grids: %d %d",i,goals_x,goals_y,goal_sort[i][0],goal_sort[i][1]);
        send_Pose.pose.position.x = goals_y;
        send_Pose.pose.position.y = goals_x;
        if(goal_sort[i][2]==1){
        	send_Pose.pose.orientation.x = 0;
	        send_Pose.pose.orientation.y = 0;
	        send_Pose.pose.orientation.z = 0;
	        send_Pose.pose.orientation.w = 1;
        }
        if(goal_sort[i][2]==2){
        	send_Pose.pose.orientation.x = 0;
	        send_Pose.pose.orientation.y = 0;
	        send_Pose.pose.orientation.z = 1;
	        send_Pose.pose.orientation.w = 1;
        }        if(goal_sort[i][2]==3){
        	send_Pose.pose.orientation.x = 0;
	        send_Pose.pose.orientation.y = 0;
	        send_Pose.pose.orientation.z = -1;
	        send_Pose.pose.orientation.w = 1;
        }        if(goal_sort[i][2]==4){
        	send_Pose.pose.orientation.x = 0;
	        send_Pose.pose.orientation.y = 0;
	        send_Pose.pose.orientation.z = -1;
	        send_Pose.pose.orientation.w = 0;
        }
	    goal.target_pose.header.frame_id = "odom";

        goal.target_pose = send_Pose;//此处记得要定义MoveBaseGoal类的goal

        ac.sendGoal(goal);

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("already reached goal!");

        }
        else{
            ROS_INFO("Failed to reach the goal!");  
        }   
  
    }
    ROS_INFO("Search finished !!!");

}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "SearchTag");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",3,RoutePlan);
    
    ros::spin();

    return 0;
}
