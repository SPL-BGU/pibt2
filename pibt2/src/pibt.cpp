#include "../include/pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";


PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr)),
      occupied_field_of_view_now(Fields()),
      occupied_field_of_view_next(Fields())
{
  solver_name = PIBT::SOLVER_NAME;
}

/**
 * @brief Calculate the number of agent groups.
 * 
 * @param P The problem instance.
 * @param k The size of each agent group.
 * @return size_t The number of agent groups.
 */
inline size_t get_num_of_agent_groups(MAPF_Instance* P, size_t k)
{
  if (P->getNum() % k != 0) {
    throw std::runtime_error(
        "The given amount of agents must be divisible by k.\n");
  }
  return P->getNum() / k;
}

bool PIBT::can_occupy(Agents& occupied, Fields& occupied_field_of_view,
                      Node* node, Agent* agent)
{
  if (nullptr == agent) {
    throw std::runtime_error("Agent must not be nullptr.\n");
  }
  // Check if node already occupied
  if (occupied[node->id] != nullptr) {
    return false;
  }

  size_t group_id = get_agent_group_id(agent->id, k);

  // Check if node is in the field of view of another agent's group
  size_t other_groups_amount = occupied_field_of_view[node->id].size() -
                            occupied_field_of_view[node->id].get_agent_group_count(group_id);
  return other_groups_amount == 0;
}

void PIBT::occupy(Agents& occupied, Fields& occupied_field_of_view, Node* node,
                  Agent* agent)
{
  if (nullptr == agent) {
    throw std::runtime_error("Agent must not be nullptr.\n");
  }
  if (occupied[node->id] != nullptr) {
    throw std::runtime_error("The node is already occupied.\n");
  }
  if (!can_occupy(occupied, occupied_field_of_view, node, agent)) {
    throw std::runtime_error("The node cannot be occupied.\n");
  }

  Nodes field_of_view = get_field_of_view(G, node, P->getFieldOfViewRadius());
  size_t group_id = get_agent_group_id(agent->id, k);

  for (const Node* current_node : field_of_view) {
    occupied_field_of_view[current_node->id].occupy(agent->id, group_id);
  }
  occupied[node->id] = agent;
}

void PIBT::deoccupy(Agents& occupied, Fields& occupied_field_of_view,
                    Node* node, Agent* agent)
{
  if (nullptr == agent) {
    throw std::runtime_error("Agent must not be nullptr.\n");
  }
  if (occupied[node->id] != agent) {
    throw std::runtime_error("The node is not occupied by the given agent.\n");
  }

  Nodes field_of_view = get_field_of_view(G, node, P->getFieldOfViewRadius());
  size_t group_id = get_agent_group_id(agent->id, k);

  for (const Node* current_node : field_of_view) {
    if (occupied_field_of_view[current_node->id].is_occupied_by_agent(agent->id)) {
      occupied_field_of_view[current_node->id].deoccupy(agent->id, group_id);
    } else {
      throw std::runtime_error(
          "The node is not occupied by the given agent.\n");
    }
  }
  if(occupied[node->id] == agent){
    // Checking this since the force flag could be set to true
    occupied[node->id] = nullptr;
  }
}

void PIBT::run()
{
  // Set the given field of view radius
  P->setFieldOfViewRadius(field_of_view_radius);
  P->setK(k);
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    A.push_back(a);
    all_agents.push_back(a);
    if (can_occupy(occupied_now, occupied_field_of_view_now, s, a)) {
      occupy(occupied_now, occupied_field_of_view_now, s, a);
    } else {
      throw std::runtime_error("Failed in initialization - the node (" + std::to_string(s->pos.x) + "," + std::to_string(s->pos.y) + ") cannot be occupied by agent " + std::to_string(i));
    }
  }
  solution.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        std::set<size_t> affected_agents = funcPIBT(a);
        if(affected_agents.empty()){
          if (a->v_next != nullptr){
            throw std::runtime_error("The agent " + std::to_string(a->id) + " has a next location, but the affected agents are empty.");
          }
          if (!can_occupy(occupied_next, occupied_field_of_view_next, a->v_now, a)){
            std::cout << "The agent " << a->id << " cannot occupy the node [id=" << a->v_now->id << "] (" << a->v_now->pos.x << "," << a->v_now->pos.y << ")" << std::endl;
            std::cout << "Current node field of view is: " << occupied_field_of_view_next[a->v_now->id].to_string() << std::endl;
            std::cout << "Now printing the entire occupied vector:" << std::endl;
            std::cout << "occupied_now: {";
            for (size_t i = 0; i < P->getNum(); i++){
              Agent * agent = A[i];
              std::cout << "agents[" << i << "].v_now = (" << agent->v_now->pos.x << "," << agent->v_now->pos.y << "), ";
            }
            std::cout << "}" << std::endl;
            std::cout << "occupied_next: {";
            for (size_t i = 0; i < P->getNum(); i++){
              Agent * agent = A[i];
              if (agent->v_next != nullptr){
                std::cout << "agents[" << i << "].v_next = (" << agent->v_next->pos.x << "," << agent->v_next->pos.y << "), ";
              }
              else{
                std::cout << "agents[" << i << "].v_next = nullptr, ";
              }
            }
            std::cout << "}" << std::endl;
            exit(1);
          }
          // Failed to plan the next location for agent a, keep it in it's current location
          occupy(occupied_next, occupied_field_of_view_next, a->v_now, a);
          a->v_next = a->v_now;
        }
      }
    }

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    // we split the deoccupy and occupy steps to avoid conflicts in the occupy step
    // deoccupy all agents
    for (auto a : A) {
      deoccupy(occupied_now, occupied_field_of_view_now, a->v_now, a);
      deoccupy(occupied_next, occupied_field_of_view_next, a->v_next, a);
    }
    // occupy next location to current location
    for (auto a : A) {
      // set next location
      config[a->id] = a->v_next;
      if (!can_occupy(occupied_now, occupied_field_of_view_now, a->v_next, a)) {
        throw std::runtime_error("The agent " + std::to_string(a->id) + " cannot occupy the node [id=" + std::to_string(a->v_next->id) + "] (" + std::to_string(a->v_next->pos.x) + "," + std::to_string(a->v_next->pos.y) + ")");
      }
      occupy(occupied_now, occupied_field_of_view_now, a->v_next, a);
      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;
    }

    // update plan
    solution.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // memory clear
  for (auto a : A) delete a;
}

std::set<size_t> PIBT::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  for (auto u : C) {
    std::set<size_t> affected_agents;
    bool fallback = false;
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue; // TODO - check if needed or included in can_occupy
    if (aj != nullptr && u == aj->v_now) continue; // Needed only when field of view is 0
    // avoid field of view conflicts
    if (!can_occupy(occupied_next, occupied_field_of_view_next, u, ai)) continue;

    // reserve
    occupy(occupied_next, occupied_field_of_view_next, u, ai);

    ai->v_next = u;

    // Move all agents that are in the field of view of the agent in their current state.
    // This includes the agent (if exists) that is currently at u. (This can happen only when field of view is 0).
    std::set<size_t> agents_viewing_u_now = occupied_field_of_view_now[u->id].get_agents_occupied();
    for (size_t agent_id : agents_viewing_u_now) {
      if (ai->id == agent_id) {
        continue;
      }

      Agent* ak = all_agents[agent_id];
      if (ak->v_next == nullptr) {
        std::set<size_t> affected_agents_k = funcPIBT(ak, ai);
        if (affected_agents_k.empty()) {
          fallback = true;
          // remove occupation for the next iterations:
          deoccupy(occupied_next, occupied_field_of_view_next, u, ai);
          ai->v_next = nullptr;
          break;  // need to fallback
        }
        else{
          // Union the sets:
          affected_agents.insert(affected_agents_k.begin(), affected_agents_k.end());
        }
      }
    }
    if (fallback) {
      for (size_t agent_id : affected_agents) {
        Agent* ak = all_agents[agent_id];
        deoccupy(occupied_next, occupied_field_of_view_next, ak->v_next, ak);
        ak->v_next = nullptr;
      }
      continue;
    }
    // success to plan next one step
    affected_agents.insert(ai->id);
    return affected_agents;
  }
  // Failed to plan the next location for agent a_i, return empty set
  return std::set<size_t>();
}

void PIBT::setParams(int argc, char* argv[])
{
  bool field_of_view_radius_provided = false;
  bool k_provided = false;
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {"field-of-view-radius", required_argument, nullptr, 'r'},
      {"mock-agents-num", required_argument, nullptr, 'k'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "dr:k:", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      case 'r':
        field_of_view_radius_provided = true;
        field_of_view_radius = std::stoi(optarg);
        break;
      case 'k':
        k = std::stoi(optarg);
        k_provided = true;
        break;
      default:
        break;
    }
  }
  if (!field_of_view_radius_provided) {
    std::cerr << "Error: The -r (or --field-of-view-radius) option is required."
              << std::endl;
    exit(1);
  }
  if (!k_provided) {
    std::cerr << "Error: The -k (or --mock-agents-num) option is required."
              << std::endl;
    exit(1);
  }
  if (k == 0) {
    std::cerr << "Error: The -k (or --mock-agents-num) option must be bigger "
                 "or equal to 1."
              << std::endl;
    exit(1);
  }
  // initialize the counters with the number of agent groups
  size_t count = get_num_of_agent_groups(P, k);
  for(int i = 0; i < G->getNodesSize(); ++i) {
    occupied_field_of_view_now.push_back(CounterWithSize(count));
    occupied_field_of_view_next.push_back(CounterWithSize(count));
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals\n"
            << "  -r --field-of-view-radius"
            << "     "
            << "radius that other agents may see each other\n"
            << "  -k --mock-agents-num"
            << "          "
            << "number of mock agents to use. This is used to group mock "
               "agents together so they can be in the same field of view\n"
            << std::endl;
}
