#include <math.h>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

#define area_size 5.0

using namespace std;

float randomizePosition()
{
  srand(6849 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

#pragma region Player
namespace jnogueira_ns
{
class Team
{
public:
  Team(string Team_name_)
  {
	Team_name = Team_name_;
	ros::NodeHandle n;
	n.getParam("/team_" + Team_name, players);
  }
  void addPlayer(string name)
  {
	players.push_back(name);
  }
  void printInfo()
  {
	ROS_INFO_STREAM("Team " << Team_name);
	for (size_t i = 0; i < players.size(); i++)
	{
	  ROS_INFO_STREAM(players[i]);
	}
  }
  bool playerBelongsToTeam(string name)
  {
	for (size_t i = 0; i < players.size(); i++)
	{
	  if (players[i] == name)
		return true;
	}
	return false;
  }
  string Team_name;

private:
  vector<string> players;
};
class Player
{
public:
  Player(string player_name_)
  {
	player_name = player_name_;
  }

  void setTeamName(string name)
  {
	if (name == "red" || name == "blue" || name == "green")
	{
	  team = name;
	}
	else
	{
	  ROS_ERROR_STREAM("Cannot set team name ");
	}
  }
  void setTeamName(int index)
  {
	if (index == 0)
	  setTeamName("red");
	else if (index == 1)
	  setTeamName("green");
	else if (index == 2)
	  setTeamName("blue");
	else
	  setTeamName("");
  }
  string getTeam()
  {
	return team;
  }
  string player_name;

private:
  int a;
  string team;
};
class MyPlayer : public Player
{
public:
  MyPlayer(string player_name_) : Player(player_name_)
  {
	team_red = (boost::shared_ptr<Team>)new Team("red");
	team_blue = (boost::shared_ptr<Team>)new Team("blue");
	team_green = (boost::shared_ptr<Team>)new Team("green");
	if (team_red->playerBelongsToTeam(player_name_))
	{
	  team_hunter = team_blue;
	  team_my = team_red;
	  team_preys = team_green;
	}
	else if (team_green->playerBelongsToTeam(player_name_))
	{
	  team_hunter = team_red;
	  team_my = team_green;
	  team_preys = team_blue;
	}
	else if (team_blue->playerBelongsToTeam(player_name_))
	{
	  team_hunter = team_green;
	  team_my = team_blue;
	  team_preys = team_red;
	}
	else
	{
	  ROS_ERROR_STREAM("Player not have team");
	}
	setTeamName(team_my->Team_name);
	ros::NodeHandle n;
	vr_marker = (boost::shared_ptr<ros::Publisher>)new ros::Publisher();
	(*vr_marker) = n.advertise<visualization_msgs::Marker>("player_names", 1);
  }
  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr make_a_play)
  {
	// ROS_INFO_STREAM("Cat_vel: " << make_a_play->cat);
	// ROS_INFO_STREAM("Dog_vel: " << make_a_play->dog);
	// ROS_INFO_STREAM("Cheetah_vel: " << make_a_play->cheetah);
	// ROS_INFO_STREAM("Turtle_vel: " << make_a_play->turtle);

	///**********
	tf::Transform transform1;

	tf::StampedTransform transform;
	try
	{
	  listener.lookupTransform("/world", "/" + player_name, ros::Time(0), transform);
	  float dx = make_a_play->turtle;
	  float dt = M_PI / 100;

	  transform1.setOrigin(tf::Vector3(dx, 0, 0));
	  tf::Quaternion q;
	  q.setRPY(0, 0, dt);
	  transform1.setRotation(q);
	  transform1 = transform * transform1;
	  double x = transform1.getOrigin().x();
	  double y = transform1.getOrigin().y();
	  x = std::min(x, area_size);
	  x = std::max(x, -area_size);
	  y = std::min(y, area_size);
	  y = std::max(y, -area_size);
	  transform1.setOrigin(tf::Vector3(x, y, 0));
	  br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", player_name));

	  visualization_msgs::Marker marker;
#pragma region name
	  marker.header.frame_id = player_name;
	  marker.header.stamp = ros::Time::now();
	  marker.ns = "basic_shapes";
	  marker.text = player_name;
	  marker.id = 0;
	  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = 0;
	  marker.pose.position.y = 0;
	  marker.pose.position.z = 0.2;
	  marker.pose.orientation.x = 0.0;
	  marker.pose.orientation.y = 0.0;
	  marker.pose.orientation.z = 0.0;
	  marker.pose.orientation.w = 1.0;

	  marker.scale.x = marker.scale.y = marker.scale.z = 0.6;

	  marker.color.r = 1.0f;
	  marker.color.g = 0.0f;
	  marker.color.b = 0.0f;
	  marker.color.a = 1.0;

	  marker.lifetime = ros::Duration();
#pragma endregion
	  visualization_msgs::Marker marker1;
#pragma regi1on forma
	  marker1.header.frame_id = player_name;
	  marker1.header.stamp = ros::Time::now();
	  marker1.ns = "basic_shapes";
	  marker1.id = 1;
	  marker1.type = visualization_msgs::Marker::SPHERE;
	  marker1.action = visualization_msgs::Marker::ADD;
	  marker1.pose.position.x = 0.3;
	  marker1.pose.position.y = 0.0;
	  marker1.pose.position.z = 0;
	  marker1.pose.orientation.x = -M_PI / 4;
	  marker1.pose.orientation.y = 0.0;
	  marker1.pose.orientation.z = 0.0;
	  marker1.pose.orientation.w = 1.0;

	  marker1.scale.x = 0.6;
	  marker1.scale.y = marker1.scale.z = 0.4;

	  marker1.color.r = 1.0f;
	  marker1.color.g = 0.0f;
	  marker1.color.b = 0.0f;
	  marker1.color.a = 1.0;

	  marker1.lifetime = ros::Duration();

#pragma endregion

	  (*vr_marker).publish(marker);
	  (*vr_marker).publish(marker1);
	}
	catch (tf::TransformException ex)
	{
	  ROS_ERROR("%s  --  define random pose", ex.what());

	  transform1.setOrigin(tf::Vector3(randomizePosition(), randomizePosition(), 0));
	  tf::Quaternion q;
	  q.setRPY(0, 0, M_PI);
	  transform1.setRotation(q);
	  br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", player_name));
	  ros::Duration(0.1).sleep();
	}
  }

  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_hunter;
  boost::shared_ptr<Team> team_my;
  boost::shared_ptr<Team> team_preys;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vr_marker;
};
}
#pragma endregion

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "player_jnogueira");
  ros::NodeHandle nn;

  jnogueira_ns::MyPlayer player("jnogueira");

  player.team_red->printInfo();
  cout << endl;
  player.team_blue->printInfo();
  cout << endl;
  player.team_green->printInfo();

  ros::Subscriber sub = nn.subscribe("/make_a_play", 100, &jnogueira_ns::MyPlayer::makeAPlayCallback, &player);

  ros::Rate l(20);
  while (ros::ok())
  {
	// ROS_INFO_STREAM("\tName: " + player.player_name + "\tTeam: " + player.getTeam());
	l.sleep();
	ros::spinOnce();
  }

  return 1;
}
