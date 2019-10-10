///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Aidan Curtis and Patrick Han
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"
// Including SimpleSetup will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>

// Except for the state space definitions...
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>


// Use placeholder namespace for arguments to bound functions.
using namespace std::placeholders;
// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
	// Cast the state to a real vector state
	auto r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();

	// Extract x, y
	double x = r2state->values[0];
	double y = r2state->values[1];

	return isValidPoint(x, y, obstacles);
}

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
	// Cast the state to a compound state
	auto cstate = state->as<ompl::base::CompoundState>();

	// Extract the real vector component (x, y)
	auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
	double x = r2state->values[0];
	double y = r2state->values[1];

	// Extract theta (SO(2))
	auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
	double theta = so2State->value;

	return isValidSquare(x, y, theta, sideLength, obstacles);
}


void planPoint(const std::vector<Rectangle> &obstacles )
{
	std::cout<<"Num obstacles inside plan "<<obstacles.size()<<std::endl;
	auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-2);  // x and y have a minimum of -2
	bounds.setHigh(2);  // x and y have a maximum of 2
	// Set the bounds on R2
	r2->setBounds(bounds);

	ompl::geometric::SimpleSetup ss(r2);
	ss.setStateValidityChecker(std::bind(isValidStatePoint, _1, obstacles));

	ompl::base::ScopedState<> start(r2);
	start[0] = -0.9;
	start[1] = -0.9;

	ompl::base::ScopedState<> goal(r2);
	goal[0] = 0.9;
	goal[1] = 0.9;

	ss.setStartAndGoalStates(start, goal);

	auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
	ss.setPlanner(planner);

	ompl::base::PlannerStatus solved = ss.solve(10.0);

	if (solved)
	{
		// print the path to screen
		std::cout << "Found solution:" << std::endl;
		ompl::geometric::PathGeometric &path = ss.getSolutionPath();
		path.interpolate(50);
		path.printAsMatrix(std::cout);

		// print path to file
		std::ofstream fout("path.txt");
		fout << "R2" << std::endl;
		path.printAsMatrix(fout);
		fout.close();
	}
	else
		std::cout << "No solution found" << std::endl;

}

void planBox(const std::vector<Rectangle> &  obstacles )
{


	ompl::base::StateSpacePtr se2;
	auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-2);  // x and y have a minimum of -2
	bounds.setHigh(2);  // x and y have a maximum of 2

	r2->setBounds(bounds);

	auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

	se2 = r2 + so2;

	ompl::geometric::SimpleSetup ss(se2);

	ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

	ompl::base::ScopedState<> start(se2);
	start[0] = -0.9;
	start[1] = -0.9;
	start[2] = 0.0;

	ompl::base::ScopedState<> goal(se2);
	goal[0] = 0.9;
	goal[1] = 0.9;
	goal[2] = 0.0;

	// set the start and goal states
	ss.setStartAndGoalStates(start, goal);

	auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
	ss.setPlanner(planner);

	// Step 6) Attempt to solve the problem within the given time (seconds)
	ompl::base::PlannerStatus solved = ss.solve(10.0);

	if (solved)
	{
		// print the path to screen
		std::cout << "Found solution:" << std::endl;
		ompl::geometric::PathGeometric &path = ss.getSolutionPath();
		path.interpolate(50);
		path.printAsMatrix(std::cout);

		// print path to file
		std::ofstream fout("path.txt");
		fout << "SE2" << std::endl;
		path.printAsMatrix(fout);
		fout.close();
	}
	else
		std::cout << "No solution found" << std::endl;
}

void makeEnvironment1(std::vector<Rectangle> &obstacles )
{
	Rectangle obstacle1;
	obstacle1.x = -0.5;
	obstacle1.y = -2;
	obstacle1.width = 0.1;
	obstacle1.height = 3.6;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 0.5;
	obstacle2.y = -1.6;
	obstacle2.width = 0.1;
	obstacle2.height = 3.6;
	obstacles.push_back(obstacle2);

	std::cout<<"After making environment "<<obstacles.size()<<std::endl;
}

void makeEnvironment2(std::vector<Rectangle> &obstacles )
{
	float side = 0.9;
	float space_factor = 1.4;
	float offset = -2;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if( !(i==0 && j==0) and !(i==2 && j==2) ){
				Rectangle obstacle;
				obstacle.x = space_factor * i + offset;
				obstacle.y = space_factor * j + offset;
				obstacle.width = side;
				obstacle.height = side;
				obstacles.push_back(obstacle);
			}
		}		
	}
}

int main(int /* argc */, char ** /* argv */)
{
	int robot, choice;
	std::vector<Rectangle> obstacles;

	do
	{
		std::cout << "Plan for: " << std::endl;
		std::cout << " (1) A point in 2D" << std::endl;
		std::cout << " (2) A rigid box in 2D" << std::endl;

		std::cin >> robot;
	} while (robot < 1 || robot > 2);

	do
	{
		std::cout << "In Environment: " << std::endl;
		std::cout << " (1) Two Vertical Bars" << std::endl;
		std::cout << " (2) Seven Squares Grid" << std::endl;

		std::cin >> choice;
	} while (choice < 1 || choice > 2);

	switch (choice)
	{
		case 1:
			std::cout<<"num obstacles before return "<<obstacles.size()<<std::endl;
			makeEnvironment1(obstacles);
			std::cout<<"num obstacles after return "<<obstacles.size()<<std::endl;
			break;
		case 2:
			makeEnvironment2(obstacles);
			break;
		default:
			std::cerr << "Invalid Environment Number!" << std::endl;
			break;
	}

	switch (robot)
	{
		case 1:
			std::cout<<"num obstacles before plan "<<obstacles.size()<<std::endl;
			planPoint(obstacles);
			break;
		case 2:
			planBox(obstacles);
			break;
		default:
			std::cerr << "Invalid Robot Type!" << std::endl;
			break;
	}

	return 0;
}
