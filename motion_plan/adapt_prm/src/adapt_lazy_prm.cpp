/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Andy Chien */

#include "motion_plan/adapt_prm/adapt_lazy_prm.h"
#if defined(__has_include) && __has_include("motion_plan/objectives/PathLengthUtilizationOptimizationObjective.h")
#include "motion_plan/objectives/PathLengthUtilizationOptimizationObjective.h"
#define USING_UTI_OPT
#else
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#endif

#if defined(__has_include) && __has_include("motion_plan/ompl_interface/parameterization/model_based_state_space.h")
#include "motion_plan/ompl_interface/parameterization/model_based_state_space.h"
#define USING_MB_SS
#endif

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/graph/copy.hpp>
#include <boost/foreach.hpp>
#include <queue>
#include <cmath>
#include <exception>

#include "motion_plan/adapt_prm/goal_visitor.hpp"

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

        /** \brief When optimizing solutions with lazy planners, this is the minimum
            number of path segments to add before attempting a new optimized solution
            extraction */
        static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;

        static const unsigned int MAX_SOLUTIONS_FOR_LAZY_OPTIMIZATION = 50;

        static const unsigned int VERTEX_CLEARING_TIMING = 25;

        static const short int UTILIZATION_THRESHOLD = -5;

        static const short int LOWEST_ATH_UTILIZATION = 25;

        static const short int MAX_VERTICES = 3125;

        static const float INVALIDITY_COST = 9765625;
    }
}

ompl::geometric::AdaptLazyPRM::AdaptLazyPRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "AdaptLazyPRM")
  , starStrategy_(starStrategy)
  , indexProperty_(boost::get(boost::vertex_index_t(), g_))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , vertexComponentProperty_(boost::get(vertex_component_t(), g_))
  , vertexValidityProperty_(boost::get(vertex_flags_t(), g_))
  , vertexUtilization_(boost::get(vertex_utilization_t(), g_))
  , edgeValidityProperty_(boost::get(edge_flags_t(), g_))
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;
    athUtilization_ = magic::LOWEST_ATH_UTILIZATION;

    Planner::declareParam<double>("range", this, &AdaptLazyPRM::setRange, &AdaptLazyPRM::getRange, "0.:1.:10000.");
    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &AdaptLazyPRM::setMaxNearestNeighbors,
                                            std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this]
                               {
                                   return getIterationCount();
                               });
    addPlannerProgressProperty("best cost REAL", [this]
                               {
                                   return getBestCost();
                               });
    addPlannerProgressProperty("milestone count INTEGER", [this]
                               {
                                   return getMilestoneCountString();
                               });
    addPlannerProgressProperty("edge count INTEGER", [this]
                               {
                                   return getEdgeCountString();
                               });
}

ompl::geometric::AdaptLazyPRM::AdaptLazyPRM(const base::PlannerData &data, bool starStrategy)
  : AdaptLazyPRM(data.getSpaceInformation(), starStrategy)
{
    athUtilization_ = magic::LOWEST_ATH_UTILIZATION;
    if (data.numVertices() > 0)
    {
        // mapping between vertex id from PlannerData and Vertex in Boost.Graph
        std::map<unsigned int, Vertex> vertices;
        // helper function to create vertices as needed and update the vertices mapping
        const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
            if (!vertices.count(vertex_index))
            {
                const auto &data_vertex = data.getVertex(vertex_index);
                Vertex graph_vertex = boost::add_vertex(g_);
                stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
                vertexValidityProperty_[graph_vertex] = VALIDITY_UNKNOWN;
                unsigned long int newComponent = componentCount_++;
                vertexComponentProperty_[graph_vertex] = newComponent;
                vertices[vertex_index] = graph_vertex;
                vertexUtilization_[graph_vertex] = magic::LOWEST_ATH_UTILIZATION;
            }
            return vertices.at(vertex_index);
        };

        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

        for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
        {
            Vertex m = getOrCreateVertex(vertex_index);
            std::vector<unsigned int> neighbor_indices;
            data.getEdges(vertex_index, neighbor_indices);
            for (const unsigned int neighbor_index : neighbor_indices)
            {
                Vertex n = getOrCreateVertex(neighbor_index);
                base::Cost weight;
                data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                const Graph::edge_property_type properties(weight);
                const Edge &edge = boost::add_edge(m, n, properties, g_).first;
                edgeValidityProperty_[edge] = VALIDITY_UNKNOWN;
                edgeUtilization_[edge] = magic::LOWEST_ATH_UTILIZATION;
                uniteComponents(m, n);
            }
            nn_->add(m);
        }
    }
}

ompl::geometric::AdaptLazyPRM::~AdaptLazyPRM() = default;

void ompl::geometric::AdaptLazyPRM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &)
        {
            return true;
        };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else
        {
            createDefaultOpt();
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
        if(opt_->getDescription().find("Utilization") != std::string::npos)
            usingUtilizationOpt_ = true;
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    sampler_ = si_->allocStateSampler();
}

void ompl::geometric::AdaptLazyPRM::resetComponent()
{
    if (boost::num_vertices(g_) > 0)
    {
        componentCount_ = 0;
        componentSize_.clear();
        std::set<Vertex> component_setted_v;
        
        boost::graph_traits<Graph>::vertex_iterator v, vend;
        for (boost::tie(v, vend) = boost::vertices(g_); v != vend; ++v)
        {
            if (!component_setted_v.count(*v))
            {
                unsigned long int newComponent = componentCount_++;
                vertexComponentProperty_[*v] = newComponent;
                componentSize_[newComponent] = 1;
                component_setted_v.insert(*v);
            }
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*v, g_); nbh != last; ++nbh)
            {
                uniteComponents(*v, *nbh);
                component_setted_v.insert(*nbh);
            }
        }
    }
}

void ompl::geometric::AdaptLazyPRM::setRange(double distance)
{
    maxDistance_ = distance;
    if (!userSetConnectionStrategy_)
        setDefaultConnectionStrategy();
    if (isSetup())
        setup();
}

void ompl::geometric::AdaptLazyPRM::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = KBoundedStrategy<Vertex>(k, maxDistance_, nn_);
    if (isSetup())
        setup();
}

void ompl::geometric::AdaptLazyPRM::setDefaultConnectionStrategy()
{
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }

    if (starStrategy_)
        connectionStrategy_ = KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = KBoundedStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS_LAZY, maxDistance_, nn_);
}

void ompl::geometric::AdaptLazyPRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::AdaptLazyPRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    tmpCost_.clear();
    pis_.restart();
}

void ompl::geometric::AdaptLazyPRM::clearValidity()
{
    std::lock_guard<std::mutex> _(graphMutex_);
    foreach (const Vertex v, boost::vertices(g_))
        vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
    foreach (const Edge e, boost::edges(g_))
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
}

void ompl::geometric::AdaptLazyPRM::clear()
{
    Planner::clear();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    componentCount_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::AdaptLazyPRM::freeMemory()
{
    std::lock_guard<std::mutex> _(graphMutex_);
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

ompl::geometric::AdaptLazyPRM::Vertex ompl::geometric::AdaptLazyPRM::addMilestone(base::State *state)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
    unsigned long int newComponent = componentCount_++;
    vertexComponentProperty_[m] = newComponent;
    componentSize_[newComponent] = 1;
    vertexUtilization_[m] = 0;

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    foreach (Vertex n, neighbors)
        if (connectionFilter_(m, n))
        {
            const base::Cost weight = costHeuristic(m, n);
            const Graph::edge_property_type properties(weight);
            const Edge &e = boost::add_edge(m, n, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            edgeUtilization_[e] = 0;
            uniteComponents(m, n);
        }

    nn_->add(m);

    return m;
}

ompl::base::PlannerStatus ompl::geometric::AdaptLazyPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    unsigned long int nvg = boost::num_vertices(g_);
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));
    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }
    
    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    explorationCondition();
    bestCost_ = opt_->infiniteCost();
    base::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    base::PathPtr bestSolution;
    bool someSolutionFound = false;
    bool fullyOptimized = false;
    bool setBounds = true;
    unsigned int optimizingComponentSegments = 0;
    unsigned int optimizingSolutions = 0;
    iterations_ = 0;

    // Grow roadmap in lazy fashion -- add vertices and edges without checking validity
    while (!ptc)
    {
        bool new_vertex_component = false;
        const long int solComponent = solutionComponent(&startGoalPair);

        if (enableExploration_)
        {
            if (++iterations_ > magic::MAX_VERTICES)
                break;
            if (bestSolution && setBounds){
                setBounds = false;
                computeAndSetBounds(bestSolution);
            }
            sampler_->sampleUniform(workState);
            Vertex addedVertex = addMilestone(si_->cloneState(workState));
            new_vertex_component = (long int)vertexComponentProperty_[addedVertex] == solComponent;
        }

        // if(solComponent == -1 || someSolutionFound)
        // {
        //     ++iterations_;
        //     sampler_->sampleUniform(workState);
        //     Vertex addedVertex = addMilestone(si_->cloneState(workState));
        //     new_vertex_component = (long int)vertexComponentProperty_[addedVertex] == solComponent;
        // }
        
        // If the start & goal are connected and we either did not find any solution
        // so far or the one we found still needs optimizing and we just added an edge
        // to the connected component that is used for the solution, we attempt to
        // construct a new solution.
        if (solComponent != -1 &&
            (!someSolutionFound || new_vertex_component))
        // if (solComponent != -1 && !someSolutionFound)
        {
            // If we already have a solution, we are optimizing. We check that we added at least
            // a few segments to the connected component that includes the previously found
            // solution before attempting to construct a new solution.
            if (someSolutionFound)
            {
                if (++optimizingComponentSegments < magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION)
                    continue;
                optimizingComponentSegments = 0;
            }
            Vertex startV = startM_[startGoalPair.first];
            Vertex goalV = goalM_[startGoalPair.second];
            base::PathPtr solution;

            solution = constructSolution(startV, goalV);

            if (solution)
            {
                someSolutionFound = true;
                base::Cost c, min_c;
                min_c = opt_->motionCostHeuristic(stateProperty_[startV], stateProperty_[goalV]); //costHeuristic(startV, goalV);
                c = base::Cost((solution->length() + 0.0001) / (min_c.value() + 0.0001));
                
                if (opt_->isSatisfied(c) || !enableExploration_)
                {
                    fullyOptimized = true;
                    bestSolution = solution;
                    bestCost_ = c;
                    break;
                }
                if (opt_->isCostBetterThan(c, bestCost_))
                {
                    bestSolution = solution;
                    bestCost_ = c;
                }
                else if (++optimizingSolutions > magic::MAX_SOLUTIONS_FOR_LAZY_OPTIMIZATION && this->isAcceptable(bestCost_))
                    break;
            }
        }
    }
    si_->freeState(workState);
    resetBounds();

    if (bestSolution && this->isAcceptable(bestCost_))
    {
        updateUtilization();
        base::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }
    removeTerminalPair(startGoalPair);
    if (enableExploration_)
    {
        if (int(++solvedCount_) % int(magic::VERTEX_CLEARING_TIMING) == 0 || boost::num_vertices(g_) > magic::MAX_VERTICES)
        {
            bool regular = int(solvedCount_) % int(magic::VERTEX_CLEARING_TIMING) == 0;
            simplifyGrapgThread_ = new std::thread(&ompl::geometric::AdaptLazyPRM::simplifyGragh, this, regular);
            simplifyGrapgThread_->detach();
        }
    }
    OMPL_INFORM("%s: Graph has %u states, best cost is %.3f", getName().c_str(), boost::num_vertices(g_), bestCost_.value());
    return bestSolution && this->isAcceptable(bestCost_)? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::AdaptLazyPRM::removeTerminalPair(std::pair<std::size_t, std::size_t> startGoalPair)
{
    nn_->remove(startM_[startGoalPair.first]);
    nn_->remove(goalM_[startGoalPair.second]);
    // Free vertex state.
    si_->freeState(stateProperty_[startM_[startGoalPair.first]]);
    si_->freeState(stateProperty_[goalM_[startGoalPair.second]]);
    // Remove the edges.
    boost::clear_vertex(startM_[startGoalPair.first], g_);
    boost::clear_vertex(goalM_[startGoalPair.second], g_);
    // Remove the vertex.
    boost::remove_vertex(startM_[startGoalPair.first], g_);
    boost::remove_vertex(goalM_[startGoalPair.second], g_);
}

void ompl::geometric::AdaptLazyPRM::uniteComponents(Vertex a, Vertex b)
{
    unsigned long int componentA = vertexComponentProperty_[a];
    unsigned long int componentB = vertexComponentProperty_[b];
    if (componentA == componentB)
        return;
    if (componentSize_[componentA] > componentSize_[componentB])
    {
        std::swap(componentA, componentB);
        std::swap(a, b);
    }
    markComponent(a, componentB);
}

void ompl::geometric::AdaptLazyPRM::markComponent(Vertex v, unsigned long int newComponent)
{
    std::queue<Vertex> q;
    q.push(v);
    while (!q.empty())
    {
        Vertex n = q.front();
        q.pop();
        unsigned long int &component = vertexComponentProperty_[n];
        if (component == newComponent)
            continue;
        if (componentSize_[component] == 1)
            componentSize_.erase(component);
        else
            componentSize_[component]--;
        component = newComponent;
        componentSize_[newComponent]++;
        boost::graph_traits<Graph>::adjacency_iterator nbh, last;
        for (boost::tie(nbh, last) = boost::adjacent_vertices(n, g_); nbh != last; ++nbh)
            q.push(*nbh);
    }
}

long int ompl::geometric::AdaptLazyPRM::solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const
{
    for (std::size_t startIndex = 0; startIndex < startM_.size(); ++startIndex)
    {
        long int startComponent = vertexComponentProperty_[startM_[startIndex]];
        for (std::size_t goalIndex = 0; goalIndex < goalM_.size(); ++goalIndex)
        {
            if (startComponent == (long int)vertexComponentProperty_[goalM_[goalIndex]])
            {
                startGoalPair->first = startIndex;
                startGoalPair->second = goalIndex;
                return startComponent;
            }
        }
    }
    return -1;
}

ompl::base::PathPtr ompl::geometric::AdaptLazyPRM::constructSolution(const Vertex &start, const Vertex &goal)
{
    // Need to update the index map here, becuse nodes may have been removed and
    // the numbering will not be 0 .. N-1 otherwise.
    unsigned long int index = 0;
    solutionM_.clear();
    solutionE_.clear();
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
        indexProperty_[*vi] = index;

    boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(g_, start,
                            [this, goal](Vertex v)
                            {
                                return costHeuristic(v, goal);
                            },
                            boost::predecessor_map(prev)
                                .distance_compare([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->isCostBetterThan(c1, c2);
                                                  })
                                .distance_combine([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->combineCosts(c1, c2);
                                                  })
                                .distance_inf(opt_->infiniteCost())
                                .distance_zero(opt_->identityCost())
                                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
    {
        // throw Exception(name_, "Could not find solution path");
        std::cerr<<"astar search could not find solution path"<<std::endl;
        return base::PathPtr();
    }

    // First, get the solution states without copying them, and check them for validity.
    // We do all the node validity checks for the vertices, as this may remove a larger
    // part of the graph (compared to removing an edge).
    std::vector<const base::State *> states(1, stateProperty_[goal]);
    std::set<Vertex> milestonesToRemove;
    bool solution_validity = true;
    for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
    {
        const base::State *st = stateProperty_[pos];
        unsigned int &vd = vertexValidityProperty_[pos];
        if ((vd & VALIDITY_TRUE) == 0)
            if (si_->isValid(st))
                vd |= VALIDITY_TRUE;
        if ((vd & VALIDITY_TRUE) == 0)
        {
            solution_validity = false;
            if (vertexUtilization_[pos] > 0) // used vertex
                tmpCost_[pos] = magic::INVALIDITY_COST;
            else
                milestonesToRemove.insert(pos); // new sampled vertex
        }
        if (solution_validity)
            states.push_back(st);
    }

    // We remove *all* invalid vertices. This is not entirely as described in the original AdaptLazyPRM
    // paper, as the paper suggest removing the first vertex only, and then recomputing the
    // shortest path. Howeve, the paper says the focus is on efficient vertex & edge removal,
    // rather than collision checking, so this modification is in the spirit of the paper.
    if (!milestonesToRemove.empty())
    {
        unsigned long int comp = vertexComponentProperty_[start];
        // Remember the current neighbors.
        std::set<Vertex> neighbors;
        for (auto it = milestonesToRemove.begin(); it != milestonesToRemove.end(); ++it)
        {
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
                if (milestonesToRemove.find(*nbh) == milestonesToRemove.end())
                    neighbors.insert(*nbh);
            // Remove vertex from nearest neighbors data structure.
            nn_->remove(*it);
            // Free vertex state.
            si_->freeState(stateProperty_[*it]);
            // Remove all edges.
            boost::clear_vertex(*it, g_);
            // Remove the vertex.
            boost::remove_vertex(*it, g_);
        }
        // Update the connected component ID for neighbors.
        for (auto neighbor : neighbors)
        {
            if (comp == vertexComponentProperty_[neighbor])
            {
                unsigned long int newComponent = componentCount_++;
                componentSize_[newComponent] = 0;
                markComponent(neighbor, newComponent);
            }
        }
        return base::PathPtr();
    }

    if (!solution_validity)
        return base::PathPtr();
        
    // start is checked for validity already
    states.push_back(stateProperty_[start]);

    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const base::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    do
    {
        Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
        unsigned int &evd = edgeValidityProperty_[e];
        if ((evd & VALIDITY_TRUE) == 0)
        {
            if (si_->checkMotion(*state, *prevState))
                evd |= VALIDITY_TRUE;
        }
        if ((evd & VALIDITY_TRUE) == 0)
        {
            // if (edgeUtilization_[e] > 0) //used edge
            //     weightProperty_[e] = ompl::base::Cost(magic::INVALIDITY_COST);
            // else
            if (edgeUtilization_[e] <= 0)
            {
                boost::remove_edge(e, g_);
                unsigned long int newComponent = componentCount_++;
                componentSize_[newComponent] = 0;
                markComponent(pos, newComponent);
            }
            solution_validity = false;
        }
        else
        {
            solutionM_.push_back(pos);
            solutionE_.push_back(e);
        }
        prevState = state;
        ++state;
        prevVertex = pos;
        pos = prev[pos];
    } while (prevVertex != pos);
    if (!solution_validity)
        return base::PathPtr();

    auto p(std::make_shared<PathGeometric>(si_));
    for (std::vector<const base::State *>::const_reverse_iterator st = states.rbegin(); st != states.rend(); ++st)
        p->append(*st);
    return p;
}

void ompl::geometric::AdaptLazyPRM::updateUtilization()
{
    if (solutionM_.size() <= 2)
        return;
    
    for (auto it = solutionM_.rbegin() + 1; it != solutionM_.rend() - 1; ++it)
    {
        short int &vertexUtilization = vertexUtilization_[*it];
        if (vertexUtilization < 1)
            vertexUtilization = 1;
        vertexUtilization = log(vertexUtilization + 1.0) / log(1.1);
        if (vertexUtilization  > athUtilization_)
            athUtilization_ = vertexUtilization ;
    }
    if (solutionE_.size() <= 2)
        return;
    for (auto it = solutionE_.begin() + 1; it != solutionE_.end() - 1; ++it)
    {
        short int &edgeUtilization = edgeUtilization_[*it];
        if (edgeUtilization < 1)
            edgeUtilization = 1;
        edgeUtilization = log(edgeUtilization + 1.0) / log(1.1);
    }
}

void ompl::geometric::AdaptLazyPRM::simplifyGragh(bool regular)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    athUtilization_ =  magic::LOWEST_ATH_UTILIZATION;
    std::set<Vertex> neighbors;
    boost::graph_traits<Graph>::vertex_iterator v, vend;
    std::set<Vertex> milestonesToRemove;
    int threshold;
    for (threshold = magic::UTILIZATION_THRESHOLD; threshold == magic::UTILIZATION_THRESHOLD || 
                     boost::num_vertices(g_) - milestonesToRemove.size() > magic::MAX_VERTICES / 2; threshold++)
    {
        for (boost::tie(v, vend) = boost::vertices(g_); v != vend; ++v)
        {
            short int *vertexUtilization = &vertexUtilization_[*v];
            if (regular && threshold == magic::UTILIZATION_THRESHOLD)
                if (--*vertexUtilization == 0 )
                    --*vertexUtilization;
                    
            if (*vertexUtilization <= threshold || (!regular && *vertexUtilization == 0)) // if not regular, remove all vertices added in that time
                milestonesToRemove.insert(*v);
            else if (*vertexUtilization > athUtilization_ && threshold == magic::UTILIZATION_THRESHOLD)
                athUtilization_ = *vertexUtilization;
        }
    }

    if (!milestonesToRemove.empty())
    {
        // unsigned long int comp = vertexComponentProperty_[start];
        // Remember the current neighbors.
        std::set<Vertex> neighbors;
        for (auto it = milestonesToRemove.begin(); it != milestonesToRemove.end(); ++it)
        {
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
                if (milestonesToRemove.find(*nbh) == milestonesToRemove.end())
                    neighbors.insert(*nbh);
            // Remove vertex from nearest neighbors data structure.
            nn_->remove(*it);
            // Free vertex state.
            si_->freeState(stateProperty_[*it]);
            // Remove all edges.
            boost::clear_vertex(*it, g_);
            // Remove the vertex.
            boost::remove_vertex(*it, g_);
        }
    }

    boost::graph_traits<Graph>::edge_iterator e, eend;
    for (boost::tie(e, eend) = boost::edges(g_); e != eend; ++e)
    {
        short int *edgeUtilization = &edgeUtilization_[*e];
        if (regular && threshold == magic::UTILIZATION_THRESHOLD)
            if (--*edgeUtilization == 0)
                --*edgeUtilization;
        if (*edgeUtilization <= threshold || (!regular && *edgeUtilization == 0)) // if not regular, remove all edges added in that time
        {
            Vertex v = boost::target(*e, g_);
            boost::remove_edge(*e--, g_);
        }
    }
    
    resetComponent();
    OMPL_INFORM("%s: Graph has %u states after simplify with threshold %d", getName().c_str(), boost::num_vertices(g_), threshold);
}

#ifdef USING_UTI_OPT
void ompl::geometric::AdaptLazyPRM::createDefaultOpt()
{
    opt_ = std::make_shared<base::PathLengthUtilizationOptimizationObjective>(si_);
}

bool ompl::geometric::AdaptLazyPRM::isAcceptable(ompl::base::Cost cost)
{
    return std::static_pointer_cast<base::PathLengthUtilizationOptimizationObjective>(opt_)
            ->isAcceptable(cost);
}

ompl::base::Cost ompl::geometric::AdaptLazyPRM::costHeuristic(Vertex u, Vertex v) const
{
    float c = 0;
    auto it = tmpCost_.find(u);
    if (it != tmpCost_.end())
        c += it->second;
    it = tmpCost_.find(v);
    if (it != tmpCost_.end())
        c += it->second;

    if (usingUtilizationOpt_ && solvedCount_ > magic::VERTEX_CLEARING_TIMING)
    {
        float uti1 = float(vertexUtilization_[u]) / float(athUtilization_);
        float uti2 = float(vertexUtilization_[u]) / float(athUtilization_);
        c += std::static_pointer_cast<base::PathLengthUtilizationOptimizationObjective>(opt_)
            ->motionCostHeuristic(stateProperty_[u], stateProperty_[v], uti1, uti2).value();
        return ompl::base::Cost(c);
    }
    else
    {
        c += opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]).value();
        return ompl::base::Cost(c);
    }
}

void ompl::geometric::AdaptLazyPRM::explorationCondition()
{
    enableExploration_ = std::static_pointer_cast<base::PathLengthUtilizationOptimizationObjective>
        (opt_)->getEnableExploration();
}

#else
void ompl::geometric::AdaptLazyPRM::createDefaultOpt()
{
    opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
}

bool ompl::geometric::AdaptLazyPRM::isAcceptable(ompl::base::Cost cost)
{
    return true;
}

ompl::base::Cost ompl::geometric::AdaptLazyPRM::costHeuristic(Vertex u, Vertex v) const
{
    float c = 0;
    auto it = tmpCost_.find(u);
    if (it != tmpCost_.end())
        c += it->second;
    it = tmpCost_.find(v);
    if (it != tmpCost_.end())
        c += it->second;

    c += opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]).value();
    return ompl::base::Cost(c);
}

void ompl::geometric::AdaptLazyPRM::explorationCondition()
{
   return;
}
#endif

#ifdef USING_MB_SS
void ompl::geometric::AdaptLazyPRM::computeAndSetBounds(const base::PathPtr p)
{
    std::vector<double> min, max;
    bool is_first = true;

    for(size_t i=0; i<std::static_pointer_cast<geometric::PathGeometric>(p)->getStateCount(); i++)
    {
        const base::State* s = std::static_pointer_cast<geometric::PathGeometric>(p)->getState(i);
        std::vector<double> values;
        si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getStateValues(s, values);
        if(is_first)
        {
            is_first = false;
            min.resize(values.size());
            max.resize(values.size());
            for(size_t j=0; j<values.size(); j++)
            {
                min[j] = 999999;
                max[j] = -999999;
            }
        }
        for(size_t j=0; j<values.size(); j++)
        {
            min[j] = std::min(min[j], values[j]);
            max[j] = std::max(max[j], values[j]);
        }
    }
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->setJointsPosBounds(min, max);
}

void ompl::geometric::AdaptLazyPRM::resetBounds()
{
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->resetJointsPosBounds();
}
#else
void ompl::geometric::AdaptLazyPRM::computeAndSetBounds(const base::PathPtr p)
{
    return;
}
void ompl::geometric::AdaptLazyPRM::resetBounds()
{
    return;
}
#endif

void ompl::geometric::AdaptLazyPRM::getPlannerData(base::PlannerData &data) const
{
    std::lock_guard<std::mutex> _(graphMutex_);
    Planner::getPlannerData(data);

    foreach (const Vertex v, boost::vertices(g_))
    {
        if(vertexUtilization_[v] > 0)
            data.addVertex(base::PlannerDataVertex(stateProperty_[v], 1));
    }
    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        if (data.vertexExists(stateProperty_[v1]) && data.vertexExists(stateProperty_[v2]))
        {
            data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

            // Add the reverse edge, since we're constructing an undirected roadmap
            data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));

            // Add tags for the newly added vertices
            data.tagState(stateProperty_[v1], (vertexValidityProperty_[v1] & VALIDITY_TRUE) == 0 ? 0 : 1);
            data.tagState(stateProperty_[v2], (vertexValidityProperty_[v2] & VALIDITY_TRUE) == 0 ? 0 : 1);
        }
    }
    data.computeEdgeWeights();
}

