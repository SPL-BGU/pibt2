/*
 * Implementation of Conflict-based Search (CBS)
 *
 * - ref
 * Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
 * Conflict-based search for optimal multi-agent pathfinding.
 * Artificial Intelligence, 219, 40–66.
 */

#pragma once
#include "solver.hpp"

class CBS : public Solver {
public:
  static const std::string SOLVER_NAME;

public:
  struct Constraint {
    int id;   // agent id
    int t;
    Node* v;  // at t
    Node* u;  // used only for swap conflict, at t-1
  };
  using Constraints = std::vector<Constraint*>;

protected:
  struct HighLevelNode {
    int id;  // high level node id
    Paths paths;
    Constraints constraints;
    int makespan;
    int soc;
    int f;   // for tie-break
    bool valid;
  };
  using CompareHighLevelNodes = std::function<bool(HighLevelNode*,
                                                   HighLevelNode*)>;

  Path getInitialPath(int id);
  Constraints getFirstConflict(const Paths& paths);
  int countConflict(const Paths& paths) const;
  int countConflict(int id, const Path& path, const Paths& _paths) const;
  virtual void setInitialHighLevelNode(HighLevelNode* n);
  virtual void invoke(HighLevelNode* h_node, int id);
  virtual Path getConstrainedPath(HighLevelNode* h_node, int id);
  virtual CompareHighLevelNodes getObjective();

public:
  CBS(Problem* _P);
  virtual ~CBS() {};

  void solve();
  static void printHelp();
};