///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Aidan Curtis and Patrick Han
//////////////////////////////////////

#include <iostream>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include "omplapp/config.h"


// Your random tree planner
#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "RTP.h"

using namespace ompl;


void preRunEvent(const base::PlannerPtr & /* planner */){
}

void postRunEvent(const base::PlannerPtr & /* planner */, tools::Benchmark::RunProperties & /* run */){
}

void benchmarkCubicles()
{
	app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;
	double runtime_limit, memory_limit;
	int run_count;

	benchmark_name = std::string("cubicles");
	std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
	std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
	setup.setRobotMesh(robot_fname);
	setup.setEnvironmentMesh(env_fname);

	base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
	start->setX(-4.96);
	start->setY(-40.62);
	start->setZ(70.57);
	start->rotation().setIdentity();

	base::ScopedState<base::SE3StateSpace> goal(start);
	goal->setX(200.49);
	goal->setY(-40.62);
	goal->setZ(70.57);
	goal->rotation().setIdentity();

	setup.setStartAndGoalStates(start, goal);
	setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
	setup.setup();

	std::vector<double> cs(3);
	cs[0] = 35;
	cs[1] = 35;
	cs[2] = 35;
	setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

	runtime_limit = 10.0;
	memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
	run_count = 50;


	tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
	tools::Benchmark b(setup, benchmark_name);

	// optionally set pre & pos run events
	b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
	b.setPostRunEvent(
		[](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });

	b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));

	// // run all planners with a uniform valid state sampler on the benchmark problem
	// setup.getSpaceInformation()->setValidStateSamplerAllocator(
	//     [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
	//         return std::make_shared<base::UniformValidStateSampler>(si);
	//     });
	// b.addExperimentParameter("sampler_id", "INTEGER", "0");
	b.benchmark(request);
	b.saveResultsToFile();
	
}




void benchmarkTwistycool()
{
	app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;
	double runtime_limit, memory_limit;
	int run_count;

    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 350.);
    bounds.setHigh(1, 250.);
    bounds.setHigh(2, -150.);
    bounds.setLow(0, 200.);
    bounds.setLow(1, 75.);
    bounds.setLow(2, -450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 60.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;


	tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
	tools::Benchmark b(setup, benchmark_name);

	// optionally set pre & pos run events
	b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
	b.setPostRunEvent(
		[](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });

	b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));

	// // run all planners with a uniform valid state sampler on the benchmark problem
	// setup.getSpaceInformation()->setValidStateSamplerAllocator(
	//     [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
	//         return std::make_shared<base::UniformValidStateSampler>(si);
	//     });
	// b.addExperimentParameter("sampler_id", "INTEGER", "0");
	b.benchmark(request);
	b.saveResultsToFile();
	
}

int main(int /* argc */, char ** /* argv */)
{
	int environment;

	do
	{
		std::cout << "Benchmark for: " << std::endl;
		std::cout << " (1) Cubicles" << std::endl;
		std::cout << " (2) Twistycool" << std::endl;

		std::cin >> environment;
	} while (environment < 1 || environment > 2);

	switch (environment)
	{
		case 1:
			benchmarkCubicles();
			break;
		case 2:
			benchmarkTwistycool();
			break;
		default:
			std::cerr << "Invalid Environment!" << std::endl;
			break;
	}

	return 0;
}
