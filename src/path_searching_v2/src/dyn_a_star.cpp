#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

namespace
{
void logIndexOutOfPool(const char *label, const Eigen::Vector3d &point, const Eigen::Vector3d &center,
                       const Eigen::Vector3i &center_idx, const Eigen::Vector3i &pool_size,
                       const double inv_step_size)
{
    const Eigen::Vector3i raw_idx = ((point - center) * inv_step_size + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_idx;
    const Eigen::Vector3i low_margin = raw_idx;
    const Eigen::Vector3i high_margin = raw_idx - (pool_size - Eigen::Vector3i::Ones());
    ROS_WARN("[Astar][debug] %s point=(%.3f, %.3f, %.3f) raw_idx=(%d,%d,%d) center=(%.3f, %.3f, %.3f) pool=(%d,%d,%d) low_margin=(%d,%d,%d) high_margin=(%d,%d,%d)",
             label,
             point.x(), point.y(), point.z(),
             raw_idx.x(), raw_idx.y(), raw_idx.z(),
             center.x(), center.y(), center.z(),
             pool_size.x(), pool_size.y(), pool_size.z(),
             low_margin.x(), low_margin.y(), low_margin.z(),
             high_margin.x(), high_margin.y(), high_margin.z());
}
}

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx))
    {
        if (debug_logging_)
        {
            logIndexOutOfPool("start_init", start_pt, center_, CENTER_IDX_, POOL_SIZE_, inv_step_size_);
        }
        return false;
    }

    if (!Coord2Index(end_pt, end_idx))
    {
        if (debug_logging_)
        {
            logIndexOutOfPool("end_init", end_pt, center_, CENTER_IDX_, POOL_SIZE_, inv_step_size_);
        }
        return false;
    }

    int occ;
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                if (debug_logging_)
                {
                    logIndexOutOfPool("start_adjusted", start_pt, center_, CENTER_IDX_, POOL_SIZE_, inv_step_size_);
                }
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));
            if (occ == -1)
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                if (debug_logging_)
                {
                    logIndexOutOfPool("end_adjusted", end_pt, center_, CENTER_IDX_, POOL_SIZE_, inv_step_size_);
                }
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;
    last_expanded_nodes_ = 0;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        if (debug_logging_)
        {
            ROS_WARN("[Astar][debug] init_err start=(%.3f, %.3f, %.3f) end=(%.3f, %.3f, %.3f) center=(%.3f, %.3f, %.3f) step=%.3f pool=(%d,%d,%d)",
                     start_pt.x(), start_pt.y(), start_pt.z(),
                     end_pt.x(), end_pt.y(), end_pt.z(),
                     center_.x(), center_.y(), center_.z(),
                     step_size_,
                     POOL_SIZE_.x(), POOL_SIZE_.y(), POOL_SIZE_.z());
        }
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        last_expanded_nodes_ = 0;
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            last_expanded_nodes_ = num_iter;
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            if (debug_logging_)
            {
                ROS_WARN("[Astar][debug] search_timeout expanded=%d start_idx=(%d,%d,%d) end_idx=(%d,%d,%d) center=(%.3f, %.3f, %.3f) step=%.3f",
                         num_iter,
                         start_idx.x(), start_idx.y(), start_idx.z(),
                         end_idx.x(), end_idx.y(), end_idx.z(),
                         center_.x(), center_.y(), center_.z(),
                         step_size_);
            }
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            last_expanded_nodes_ = num_iter;
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    if (debug_logging_)
    {
        ROS_WARN("[Astar][debug] search_failed expanded=%d start_idx=(%d,%d,%d) end_idx=(%d,%d,%d) center=(%.3f, %.3f, %.3f) step=%.3f pool=(%d,%d,%d)",
                 num_iter,
                 start_idx.x(), start_idx.y(), start_idx.z(),
                 end_idx.x(), end_idx.y(), end_idx.z(),
                 center_.x(), center_.y(), center_.z(),
                 step_size_,
                 POOL_SIZE_.x(), POOL_SIZE_.y(), POOL_SIZE_.z());
    }

    last_expanded_nodes_ = num_iter;
    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
