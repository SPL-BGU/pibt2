/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"
#include <set>

/**
 * @brief This class allows to count the number of agent groups that occupy a node.
 * Also, for each agent group, it allows to count the number of agents from that group that occupy a node.
 * 
 * @note It is used for the field of view of agents.
 */
class CounterWithSize {
private:
  std::vector<size_t> agent_groups;
  std::set<size_t> agents_occupied;
  size_t _size;
  size_t agent_groups_count;
public:
  CounterWithSize(size_t _agent_groups_count) : agent_groups(_agent_groups_count, 0), agents_occupied(), _size(0), agent_groups_count(_agent_groups_count) {}

  /**
   * @brief The size of the counter for all agent groups.
   * 
   * @return size_t The size of the counter.
   */
  size_t size() const { return _size; }

  /**
   * @brief Get the agents occupied set.
   * 
   * @return std::set<size_t> The agents that see the node (in their field of view).
   */
  std::set<size_t> get_agents_occupied() const { return agents_occupied; }

  /**
   * @brief Get the number of agents in the given agent group that occupy the node.
   * 
   * @param i The agent group id.
   * @return size_t The number of agents in the given agent group that occupy the node.
   */
  size_t get_agent_group_count(size_t i) const { 
    if (i >= agent_groups_count) {
      throw std::runtime_error("Index out of bounds.");
    }
    return agent_groups[i]; 
  }

  /**
   * @brief Checks if the field of view is occupied by the given agent.
   * 
   * @param agent_id The agent id.
   * @return true If occupied by the agent.
   * @return false Otherwise.
   */
  bool is_occupied_by_agent(size_t agent_id) const {
    return agents_occupied.count(agent_id) > 0;
  }

  /**
   * @brief Increase the number of agents in the given agent group that occupy the node by one.
   * 
   * @param i The agent group id.
   */
  void occupy(size_t agent_id, size_t agent_group_id) {
    if (agent_group_id >= agent_groups_count) {
      throw std::runtime_error("Index out of bounds.");
    }
    if (agents_occupied.count(agent_id) > 0) {
      throw std::runtime_error("The agent is already occupying the node.");
    }
    ++_size;
    ++agent_groups[agent_group_id];
    agents_occupied.insert(agent_id);
  }

  /**
   * @brief Decrease the number of agents in the given agent group that occupy the node by one.
   * 
   * @param i The agent group id.
   */
  void deoccupy(size_t agent_id, size_t agent_group_id) {
    if (agent_group_id >= agent_groups_count) {
      throw std::runtime_error("Index out of bounds.");
    }
    if(agents_occupied.count(agent_id) == 0) {
      throw std::runtime_error("The agent is not occupying the node.");
    }
    if (agent_groups[agent_group_id] == 0) {
      throw std::runtime_error("Cannot decrease from index " + std::to_string(agent_group_id) + " because it is already 0.");
    }
    --_size;
    --agent_groups[agent_group_id];
    agents_occupied.erase(agent_id);
  }

  std::string to_string() const {
    std::string str = "groups: [";
    for (size_t i = 0; i < agent_groups.size(); ++i) {
      str += std::to_string(agent_groups[i]) + ", ";
    }
    str += "], agents: [";
    for (size_t agent_id : agents_occupied) {
      str += std::to_string(agent_id) + ", ";
    }
    str += "]";
    return str;
  }

};

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
  };
  using Agents = std::vector<Agent*>;

  using Fields = std::vector<CounterWithSize>;

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  // All agents
  Agents all_agents;

  // <node-id, set<agent_group_id>>, whether the node is in the field of view of an agent or not.
  // A node can be in the field of view of several agent groups.
  // Nodes that are not in the field of view of an agent are empty sets.
  // Work as reservation table.
  Fields occupied_field_of_view_now;
  Fields occupied_field_of_view_next;

  // option
  bool disable_dist_init = false;
  int field_of_view_radius = 0; // field_of_view_radius - 
          // each agent must keep away from other agents by at least this radius.
  size_t k;  // k - amount of mock agents used for each agent.

  /**
   * @brief Actual priority inheritance with backtracking algorithm.
   * 
   * @param ai The agent to plan the next location for.
   * @param aj The agent that has higher priority than ai, and ai needs to replan because of aj. If ai is the highest priority agent, then aj is nullptr.
   * @return std::set<size_t> The agents that were affected from the planning of the agent ai. If invalid plan for ai, returns an empty set.
   */
  std::set<size_t> funcPIBT(Agent* ai, Agent* aj = nullptr);

  /**
   * @brief Occupies the entire field of view of the given node to be of the given agent.
   * 
   * @note In order to de-occupy, call the deoccupy function.
   * 
   * @param occupied The occupied nodes vector to occupy at.
   * @param occupied_field_of_view The occupied field of view vector to occupy at.
   * @param node The node to occupy it's field of view.
   * @param agent The agent to occupy inside the field of view.
   * 
   */
  void occupy(Agents & occupied, Fields & occupied_field_of_view, Node * node, Agent * agent);

  /**
   * @brief De-occupies the entire field of view of the given node to be of the given agent.
   * 
   * @param occupied The occupied nodes vector to de-occupy at.
   * @param occupied_field_of_view The occupied field of view vector to de-occupy at.
   * @param node The node to de-occupy it's field of view.
   * @param agent The agent to de-occupy inside the field of view.
   * 
   */
  void deoccupy(Agents & occupied, Fields & occupied_field_of_view, Node * node, Agent * agent);

  /**
   * @brief Checks if the given agent can occupy the given node.
   * 
   * @note A node can be occupied if it is not in the field of view of some other agent's group, and not already occupied.
   * 
   * @param occupied The occupied nodes vector to check if the node is occupied.
   * @param occupied_field_of_view The occupied field of view vector to check if the node is in the field of view of another agent's group.
   * @param node The node to check if it can be occupied.
   * @param agent The agent to check if can occupy the node.
   * @return true If the agent can occupy the node.
   * @return false Otherwise.
   */
  bool can_occupy(Agents & occupied, Fields & occupied_field_of_view, Node * node, Agent * agent);

  // main
  void run();

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
