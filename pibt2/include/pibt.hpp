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

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  // option
  bool disable_dist_init = false;
  int field_of_view_radius = 0; // field_of_view_radius - 
          // each agent must keep away from other agents by at least this radius.

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  /**
   * @brief Occupies the entire field of view of the given node to be of the given agent.
   * 
   * @note In order to un-occupy, just insert nullptr as the given agent.
   * 
   * @param occupied The occupied nodes vector to occupy at.
   * @param node The node to occupy it's field of view.
   * @param agent The agent to occupy inside the field of view.
   */
  void occupy(Agents & occupied, Node * node, Agent * agent);

  // main
  void run();

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
