/**
 * @file mp_map_util.h
 * @brief motion primitive map util
 */
#include <planner/env_decomp.h>
#include <planner/mp_base_util.h>

/**
 * @brief Motion primitive planner using decomposition
 */
class MPDecompUtil : public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPDecompUtil(bool verbose);
    ///Planning from start to goal
    bool plan(const Waypoint &start, const Waypoint &goal);
    ///Set map util
    void setMap(const vec_Vec3f& obs, decimal_t r);
    ///Get polyhedra
    Polyhedra getPolyhedra();
    vec_Ellipsoid getEllipsoids();
};
