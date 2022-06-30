// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "agent.h"

#include <cassert>

const char *state_labels[STATE_COUNT] = {
/* no_state */                   "No state",
/* forced_move_state */          "Forced move",
/* receive_reward_state */       "Receive reward",
/* initiate_navigation_state */  "Initiate navigation",
/* approach_subgoal_state */     "Approach subgoal",
/* topological_step_state */     "Topological step",
/* replay_episode_state */       "Replay episode",
/* exploration_state */          "Exploration",
};

void ForcedMoveState::hook(Agent *agent)
{
    agent->model->input.motor_mode = forced_mode;
    agent->model->input.motor_tuning = 0.1;
    agent->model->input.motor_offset = atan2(
        agent->input.goto_y - agent->input.y,
        agent->input.goto_x - agent->input.x);

    double goto_distance = std::sqrt(
        std::pow(agent->input.goto_x - agent->input.x, 2) +
        std::pow(agent->input.goto_y - agent->input.y, 2));
    if (goto_distance < 2 * DISTANCE_PER_TIMESTEP) {
        agent->next_state = no_state;
    }
}

void ReceiveRewardState::hook(Agent *agent)
{
    agent->model->input.motor_mode = halt_mode;
    agent->model->place_graph->input.form_place_cells = true;
    agent->model->place_graph->input.save_reward = true;
    agent->next_state = no_state;
}

void InitiateNavigationState::hook(Agent *agent)
{
    if (!agent->perform_topological_navigation) {
        agent->model->place_graph->input.reset_replay_to = goal_node;
    } else {
        agent->model->place_graph->input.reset_replay_to = agent_node;
        agent->model->place_graph->input.propagate_replay_towards = goal_node;
    }
    agent->next_state = approach_subgoal_state;
}

void ApproachSubgoalState::hook(Agent *agent)
{
    if (agent->model->place_graph->output.at_subgoal) {
        agent->next_state = topological_step_state;
    } else if (agent->model->output.halted) {
        agent->model->input.confidence_threshold = agent->replay_confidence_threshold;
        agent->next_state = replay_episode_state;
    } else {
        agent->next_state = approach_subgoal_state;
    }
}

void NoResumeApproachSubgoalState::hook(Agent *agent)
{
    if (agent->model->place_graph->output.at_subgoal) {
        agent->next_state = topological_step_state;
    } else if (agent->model->output.halted) {
        agent->model->place_graph->input.reset_replay_to = goal_node;
        agent->model->input.motor_tuning = agent->replay_motor_tuning;
        agent->model->input.confidence_threshold = agent->replay_confidence_threshold;
        agent->next_state = replay_episode_state;
    } else {
        agent->next_state = approach_subgoal_state;
    }
}

void NoTopoApproachSubgoalState::hook(Agent *agent)
{
    if (agent->model->place_graph->output.at_subgoal) {
        agent->next_state = initiate_navigation_state;
    } else if (agent->model->output.halted) {
        agent->model->input.confidence_threshold = agent->replay_confidence_threshold;
        agent->next_state = replay_episode_state;
    } else {
        agent->next_state = approach_subgoal_state;
    }
}

void TopologicalStepState::hook(Agent *agent)
{
    agent->model->place_graph->input.reset_replay_to = agent_node;
    agent->model->place_graph->input.propagate_replay_towards = goal_node;
    if (Random::uniform() < agent->topological_reset_probability) {
        agent->next_state = initiate_navigation_state;
    } else {
        agent->next_state = approach_subgoal_state;
    }
}

void ReplayEpisodeState::hook(Agent *agent)
{
    if (!agent->model->output.halted) {
        agent->next_state = approach_subgoal_state;
    } else if (agent->model->place_graph->output.replay_terminated) {
        agent->model->place_graph->input.weaken_synapse = true;
        agent->model->input.motor_mode = last_heading_mode;
        agent->model->input.motor_offset = M_PI;
        agent->next_state = exploration_state;
    } else {
        agent->model->input.motor_tuning = agent->replay_motor_tuning;
        agent->model->input.confidence_threshold = agent->replay_confidence_threshold;
        agent->model->place_graph->input.propagate_replay_towards = agent_node;
        agent->next_state = replay_episode_state;
    }
}

void ExplorationState::hook(Agent *agent)
{
    agent->model->input.motor_mode = last_heading_mode;
    agent->model->input.motor_tuning = agent->exploration_motor_tuning;
    agent->model->input.motor_offset = 0.02 * Random::normal();
    if (Random::uniform() < agent->exploration_end_probability) {
        agent->next_state = initiate_navigation_state;
    } else {
        agent->next_state = exploration_state;
    }
}

Agent::Agent(Model *model, std::string label,
        StateImplementation *initiate_navigation_state_impl,
        StateImplementation *approach_subgoal_state_impl,
        StateImplementation *topological_step_state_impl,
        StateImplementation *replay_episode_state_impl,
        StateImplementation *exploration_state_impl)
    : model(model), label(label)
{
    this->approach_motor_tuning = 0.75;
    this->replay_motor_tuning = 0.1;
    this->exploration_motor_tuning = 0.1;
    this->approach_confidence_threshold = 0.05;
    this->replay_confidence_threshold = 0.2;
    this->form_place_cells = true;
    this->perform_topological_navigation = false;
    this->exploration_end_probability = 0.003;
    this->topological_reset_probability = 0.05;

    this->state_impl[forced_move_state] = new ForcedMoveState();
    this->state_impl[receive_reward_state] = new ReceiveRewardState();

    this->state_impl[initiate_navigation_state] = initiate_navigation_state_impl;
    this->state_impl[approach_subgoal_state] = approach_subgoal_state_impl;
    this->state_impl[topological_step_state] = topological_step_state_impl;
    this->state_impl[replay_episode_state] = replay_episode_state_impl;
    this->state_impl[exploration_state] = exploration_state_impl;
}

void Agent::execute()
{
    this->model->input.heading = this->input.heading;
    this->model->input.speed = this->input.speed;
    this->model->input.motor_mode = grid_decoder_mode;
    this->model->input.motor_tuning = this->approach_motor_tuning;
    this->model->input.motor_offset = 0.0;
    this->model->input.confidence_threshold = this->approach_confidence_threshold;

    this->model->place_graph->input.x = this->input.x;
    this->model->place_graph->input.y = this->input.y;
    this->model->place_graph->input.reward_id = this->input.reward_id;
    this->model->place_graph->input.save_reward = false;
    this->model->place_graph->input.form_place_cells = this->form_place_cells;
    this->model->place_graph->input.weaken_synapse = false;
    this->model->place_graph->input.reset_replay_to = maintain_current_node;
    this->model->place_graph->input.propagate_replay_towards = maintain_current_node;

    this->next_state = this->active_state;
    StateImplementation *current_state_impl = this->state_impl[this->active_state];
    assert(current_state_impl);

    current_state_impl->hook(this);
    this->model->simulate_timestep();

    this->output.heading = this->model->output.heading;
    this->output.speed = this->model->output.speed;
    this->output.halted = this->model->output.halted;

    this->next_previous_state = this->previous_state;
    this->previous_state = this->active_state;
    this->active_state = this->next_state;

    if (this->state_impl[this->active_state] == nullptr) {
        this->active_state = no_state;
    }
}

VectorAgent::VectorAgent(Model *model)
    : Agent(model, "Purely vector-navigating agent",
            new InitiateNavigationState(),
            new ApproachSubgoalState(),
            nullptr, nullptr, nullptr)
{
    this->approach_motor_tuning = 0.1;
}

NoResumeCombinedStrictAgent::NoResumeCombinedStrictAgent(Model *model)
    : Agent(model, "Combined vector-place agent, strict replay, no resuming replays",
            new InitiateNavigationState(),
            new NoResumeApproachSubgoalState(),
            new TopologicalStepState(),
            new ReplayEpisodeState(),
            new ExplorationState())
{
    this->replay_confidence_threshold = 0.9;
}

NoTopoCombinedStrictAgent::NoTopoCombinedStrictAgent(Model *model)
    : Agent(model, "Combined vector-place agent, strict replay, no topological navigation",
            new InitiateNavigationState(),
            new NoTopoApproachSubgoalState(),
            new TopologicalStepState(),
            new ReplayEpisodeState(),
            new ExplorationState())
{
    this->replay_confidence_threshold = 0.9;
}

UnifiedAgent::UnifiedAgent(Model *model, std::string label)
    : Agent(model, label,
            new InitiateNavigationState(),
            new ApproachSubgoalState(),
            new TopologicalStepState(),
            new ReplayEpisodeState(),
            new ExplorationState()) {}

DeflectAgent::DeflectAgent(Model *model)
    : UnifiedAgent(model, "Vector-navigating agent with obstacle deflection")
{
    this->form_place_cells = false;
}

PlaceAgent::PlaceAgent(Model *model)
    : UnifiedAgent(model, "Purely topological agent")
{
    this->perform_topological_navigation = true;
}

CombinedAgent::CombinedAgent(Model *model)
    : UnifiedAgent(model, "Combined vector-place agent") {}

CombinedNarrowAgent::CombinedNarrowAgent(Model *model)
    : UnifiedAgent(model, "Combined vector-place agent, sunburst version")
{
    this->approach_motor_tuning = 0.1;
    this->exploration_end_probability = 0.0005;
}

CombinedStrictAgent::CombinedStrictAgent(Model *model)
    : UnifiedAgent(model, "Combined vector-place agent, exaggerated traits")
{
    this->replay_confidence_threshold = 0.9;
    this->topological_reset_probability = 0.25;
}
