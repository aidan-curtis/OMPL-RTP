///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Aidan Curtis and Patrick Han
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
	namespace geometric
	{

		class RTP : public base::Planner
		{
		public:
			/** \brief Constructor */
			RTP(const base::SpaceInformationPtr &si);

			~RTP() override;

			void getPlannerData(base::PlannerData &data) const override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

			void clear() override;


			/** \brief Set the goal bias

				In the process of randomly selecting states in
				the state space to attempt to go towards, the
				algorithm may in fact choose the actual goal state, if
				it knows it, with some probability. This probability
				is a real number between 0.0 and 1.0; its value should
				usually be around 0.05 and should not be too large. It
				is probably a good idea to use the default value. */
			void setGoalBias(double goalBias)
			{
				goalBias_ = goalBias;
			}

			/** \brief Get the goal bias the planner is using */
			double getGoalBias() const
			{
				return goalBias_;
			}

			void setRange(double distance)
			{
				maxDistance_ = distance;
			}

			double getRange() const
			{
				return maxDistance_;
			}
			
			void setup() override;



		protected:
			/** \brief Representation of a motion

				This only contains pointers to parent motions as we
				only need to go backwards in the tree. */
			class Motion // A motion contains a current state and its parent motion in the tree, basically an edge in the graph
			{
			public:
				Motion() = default;

				/** \brief Constructor that allocates memory for the state */
				Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
				{
				}

				~Motion() = default;

				/** \brief The state contained by the motion */
				base::State *state{nullptr};

				/** \brief The parent motion in the exploration tree */
				Motion *parent{nullptr};
			};

			/** \brief Free the memory allocated by this planner */
			void freeMemory();

			/** \brief Compute distance between motions (actually distance between contained states) */
			double distanceFunction(const Motion *a, const Motion *b) const
			{
				return si_->distance(a->state, b->state);
			}

			/** \brief State sampler */
			base::StateSamplerPtr sampler_;

			/** \brief A simple vector-based datastructure containing the tree of motions */
			std::vector<Motion *> allMotions; // Holds pointers to motions 

			/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
			 * available) */
			double goalBias_{.05};

			/** \brief The maximum length of a motion to be added to a tree */
			double maxDistance_{0.};

			/** \brief The random number generator */
			RNG rng_;

			/** \brief The most recent goal motion.  Used for PlannerData computation */
			Motion *lastGoalMotion_{nullptr};

		};
	}  // namespace geometric
}  // namespace ompl

#endif
