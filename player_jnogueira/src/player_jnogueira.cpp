#include <math.h>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

#define area_size 5.0
#define max_rotation_vel M_PI / 30

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
  vector<string> getPlayers()
  {
	return players;
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
	tf::Transform transform1;
	transform1.setOrigin(tf::Vector3(randomizePosition(), randomizePosition(), 0));
	tf::Quaternion q;
	q.setRPY(0, 0, M_PI);
	transform1.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", player_name));
  }

  std::tuple<float, float> getDistancePlayer(string other_player)
  {
	tf::StampedTransform transform;
	try
	{
	  listener.lookupTransform(player_name, other_player, ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
	  ROS_ERROR("%s", ex.what());
	  ros::Duration(0.1).sleep();
	  return { 10000.0, 0.0 };
	}

	float x = transform.getOrigin().x();
	float y = transform.getOrigin().y();

	float distance = sqrt(x * x + y * y);
	float angl = atan2(y, x);

	return std::tuple<float, float>{ distance, angl };
  }

  tuple<string, float, float> target(rws2019_msgs::MakeAPlayConstPtr make_a_play)
  {
	string player_target = "";
	float val, a;
	vector<string> p = team_preys->getPlayers();
	for (size_t i = 0; i < p.size(); i++)
	{
	  tuple<float, float> tmp = getDistancePlayer(p[i]);
	  float d = (float)std::get<0>(tmp);
	  float a = (float)std::get<1>(tmp);
	  float dx = make_a_play->turtle;
	  if ((a > -M_PI / 2) && (a < M_PI / 2))
	  {
		dx = dx * cos(M_PI / 2 - a);
	  }

	  // d += dx * (a / max_rotation_vel);

	  if (val > d)
	  {
		player_target = p[i];
		val = (float)std::get<0>(tmp);
		a = (float)std::get<1>(tmp);
	  }
	}
	return tuple<string, float, float>{ player_target, val, a };
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr make_a_play)
  {
	// ROS_INFO_STREAM("Cat_vel: " << make_a_play->cat);
	// ROS_INFO_STREAM("Dog_vel: " << make_a_play->dog);
	// ROS_INFO_STREAM("Cheetah_vel: " << make_a_play->cheetah);
	// ROS_INFO_STREAM("Turtle_vel: " << make_a_play->turtle);

	///**********

	tf::StampedTransform transform;
	try
	{
	  listener.lookupTransform("/world", "/" + player_name, ros::Time(0), transform);

	  tuple<string, float, float> catch_ = target(make_a_play);

	  float dx = (float)std::get<1>(catch_) * 0;
	  float a = (float)std::get<2>(catch_) * 0;

	  float dx_max = make_a_play->turtle;
	  dx > dx_max ? dx = dx_max : dx = dx;

	  double amax = M_PI / 30;
	  fabs(a) > fabs(amax) ? a = amax * a / fabs(a) : a = a;

	  // STEP 3: define local movement
	  tf::Transform T1;
	  T1.setOrigin(tf::Vector3(dx, 0.0, 0.0));
	  tf::Quaternion q;
	  q.setRPY(0, 0, a);
	  T1.setRotation(q);

	  // STEP 4: define global movement
	  tf::Transform Tglobal = transform * T1;
	  br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

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
