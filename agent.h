// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef AGENT_H_INCLUDED
#define AGENT_H_INCLUDED

#include "main.h"
#include "model.h"

enum State {
    no_state,

    forced_move_state,
    receive_reward_state,

    initiate_navigation_state,
    approach_subgoal_state,
    topological_step_state,
    replay_episode_state,
    exploration_state,

    STATE_COUNT
};

extern const char *state_labels[STATE_COUNT];

class StateImplementation { public: virtual void hook(class Agent *agent) = 0; };

class ForcedMoveState : public StateImplementation { public: void hook(Agent *agent); };
class ReceiveRewardState : public StateImplementation { public: void hook(Agent *agent); };
class InitiateNavigationState : public StateImplementation { public: void hook(Agent *agent); };
class ApproachSubgoalState : public StateImplementation { public: void hook(Agent *agent); };
class NoResumeApproachSubgoalState : public StateImplementation { public: void hook(Agent *agent); };
class NoTopoApproachSubgoalState : public StateImplementation { public: void hook(Agent *agent); };
class TopologicalStepState : public StateImplementation { public: void hook(Agent *agent); };
class ReplayEpisodeState : public StateImplementation { public: void hook(Agent *agent); };
class ExplorationState : public StateImplementation { public: void hook(Agent *agent); };

class Agent
{
    public:
        Agent(Model *model, std::string label,
            StateImplementation *initiate_navigation_state_impl,
            StateImplementation *approach_subgoal_state_impl,
            StateImplementation *topological_step_state_impl,
            StateImplementation *replay_episode_state_impl,
            StateImplementation *exploration_state_impl);
        void execute();

        // Input values

        struct {
            double x, y;
            double heading, speed;
            double goto_x, goto_y;
            int reward_id;
        } input;

        // Output values

        struct {
            double heading;
            double speed;
            bool halted;
        } output;

        // Parameters

        double approach_motor_tuning;
        double replay_motor_tuning;
        double exploration_motor_tuning;
        double approach_confidence_threshold;
        double replay_confidence_threshold;
        bool form_place_cells;
        bool perform_topological_navigation;
        double exploration_end_probability;
        double topological_reset_probability;

        // States and other pointers

        std::string label;
        Model *model;
        State active_state = no_state;
        State next_state = no_state;
        State previous_state = no_state;
        State next_previous_state = no_state;
        StateImplementation *state_impl[STATE_COUNT] = { nullptr };
};

class VectorAgent : public Agent { public: VectorAgent(Model *model); };
class NoResumeCombinedStrictAgent : public Agent { public: NoResumeCombinedStrictAgent(Model *model); };
class NoTopoCombinedStrictAgent : public Agent { public: NoTopoCombinedStrictAgent(Model *model); };

class UnifiedAgent : public Agent { public: UnifiedAgent(Model *model, std::string label); };

class DeflectAgent : public UnifiedAgent { public: DeflectAgent(Model *model); };
class PlaceAgent : public UnifiedAgent { public: PlaceAgent(Model *model); };
class CombinedAgent : public UnifiedAgent { public: CombinedAgent(Model *model); };
class CombinedNarrowAgent : public UnifiedAgent { public: CombinedNarrowAgent(Model *model); };
class CombinedStrictAgent : public UnifiedAgent { public: CombinedStrictAgent(Model *model); };

#endif
