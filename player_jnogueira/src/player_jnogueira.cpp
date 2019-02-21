#include <ros/ros.h>
#include <iostream>
#include <vector>

using namespace std;

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
  }

  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_hunter;
  boost::shared_ptr<Team> team_my;
  boost::shared_ptr<Team> team_preys;
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

  ros::Rate l(1);
  while (ros::ok())
  {
	ROS_INFO_STREAM("\tName: " + player.player_name + "\tTeam: " + player.getTeam());
	l.sleep();
  }

  return 1;
}
