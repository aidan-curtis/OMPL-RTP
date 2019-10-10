///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Aidan Curtis and Patrick Han
//////////////////////////////////////

#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"


ompl::geometric::RTP::RTP(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "RTP") {
	specs_.approximateSolutions = true;
	specs_.directed = true;

	Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");

	Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");

}

ompl::geometric::RTP::~RTP() {
	freeMemory();
}


void ompl::geometric::RTP::clear() {
	Planner::clear();
	sampler_.reset();
	freeMemory();
	allMotions.clear(); /// Clear our vector that holds the motions of our tree
	lastGoalMotion_ = nullptr;
}

void ompl::geometric::RTP::freeMemory()
{
	allMotions.clear();
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
	checkValidity();
	ompl::base::Goal *goal = pdef_->getGoal().get(); // Extract the goal state from our problem defintion (pdef pointer)
	auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal); // Cast to a GoalSampleableRegion

	// I think this basically builds the tree initially from input states
	while (const ompl::base::State *st = pis_.nextStart()) //pis_ : planner input states, return next valid start state
	{
		auto *motion = new Motion(si_); // create a Motion ptr, initialize Motion with state information pointer
		si_->copyState(motion->state, st); // Copy the st extracted into si_
		allMotions.push_back(motion); // Add this motion to the tree
	}

	if (allMotions.size() == 0) // If our tree is empty, that means we couldn't initialize any start states
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return ompl::base::PlannerStatus::INVALID_START;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), allMotions.size());
	// Initialize both approximate and exact solution pointers
	Motion *solution = nullptr; 
	Motion *approxsol = nullptr;
	double approxdif = std::numeric_limits<double>::infinity();
	auto *rmotion = new Motion(si_);
	ompl::base::State *rstate = rmotion->state; // rstate : holds our sampled random state

	while (!ptc)
	{
		/* sample random state (with goal biasing) */
		if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
			goal_s->sampleGoal(rstate); // With small probability, sample the goal region
		else
			sampler_->sampleUniform(rstate); // Otherwise, sample a state uniformly

		/* Choose a random state from the tree */
		int randI = std::rand() % allMotions.size();
		Motion *nmotion = allMotions[randI];

		if (si_->checkMotion(nmotion->state, rstate)) // check if path between two states is valid (takes pointers to State objects)
		{
				auto *motion = new Motion(si_); // allocate memory for a state
				si_->copyState(motion->state, rstate); // Copy random state into our new motion's state
				motion->parent = nmotion; // Set the newly copied random state's parent as our nearest motion
				allMotions.push_back(motion); // Push the new motion onto the tree

				nmotion = motion;

			double dist = 0.0;
			bool sat = goal->isSatisfied(nmotion->state, &dist);
			if (sat) // found exact solution
			{
				approxdif = dist;
				solution = nmotion;
				break;
			}
			if (dist < approxdif) // approximate solution
			{
				approxdif = dist;
				approxsol = nmotion;
			}
		}
	}

	bool solved = false;
	bool approximate = false;
	if (solution == nullptr)
	{
		solution = approxsol;
		approximate = true;
	}

	if (solution != nullptr) // exact solution
	{
		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<Motion *> mpath; // mpath stores our Motion pointers for the path
		while (solution != nullptr)
		{
			mpath.push_back(solution);
			solution = solution->parent; // Keep moving up the tree of motions until nullptr (root)
		}

		/* set the solution path */
		auto path(std::make_shared<PathGeometric>(si_));
		for (int i = mpath.size() - 1; i >= 0; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(path, approximate, approxdif, getName());
		solved = true;
	}

	if (rmotion->state != nullptr)
		si_->freeState(rmotion->state);
	delete rmotion;

	OMPL_INFORM("%s: Created %u states", getName().c_str(), allMotions.size());

	return {solved, approximate};
}

void ompl::geometric::RTP::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);
}


void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	if (lastGoalMotion_ != nullptr)
		data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

	for (auto &motion : allMotions)
	{
		if (motion->parent == nullptr)
			data.addStartVertex(ompl::base::PlannerDataVertex(motion->state));
		else
			data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state), ompl::base::PlannerDataVertex(motion->state));
	}
}

