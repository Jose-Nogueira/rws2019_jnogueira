#include <ros/ros.h>
#include <iostream>
#include <vector>

using namespace std;

#pragma region Player
namespace jnogueira_ns
{
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
	  cerr << "Cannot set team name " << name << endl;
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
  MyPlayer(string player_name_, string team) : Player(player_name_)
  {
	setTeamName(team);
  }
};
class Team
{
public:
  Team(string Team_name_)
  {
	Team_name = Team_name_;
  }
  void addPlayer(string name)
  {
	players.push_back(name);
  }
  void printInfo()
  {
	cout << "Team " << Team_name << endl;
	for (size_t i = 0; i < players.size(); i++)
	{
	  cout << players[i] << endl;
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
}
#pragma endregion

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "player_jnogueira");
  ros::NodeHandle nn;

  jnogueira_ns::MyPlayer player("José Nogueira", "red");

  jnogueira_ns::Team team("red");
  team.addPlayer("José Nogueira");
  team.addPlayer("wesxdcfghbjkm");

  cout << "Name: " << player.player_name << "\tTeam: " << player.getTeam() << endl;

  string name;
  cin >> name;
  cout << name;
  if (team.playerBelongsToTeam(name))
	cout << "sim" << endl;
  else
	cout << "não" << endl;

  team.printInfo();

  ros::Rate l(1);
  int i = 0;
  while (ros::ok())
  {
	cout << i++ << "\tName: " << player.player_name << "\tTeam: " << player.getTeam() << endl;
	l.sleep();
  }

  return 1;
}
