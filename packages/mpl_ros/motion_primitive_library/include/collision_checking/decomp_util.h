/**
 * @file decomp_util.h
 * @brief Safe Flight Corridor util class
 */
#ifndef SFC_UTIL_H
#define SFC_UTIL_H
#include <primitive/primitive.h>
#include <decomp_util/seed_decomp.h>

/**
 * @brief Collision checking inside a Safe Flight Corridor (SFC)
 *
 * SFC is an ordered collection of convex polyhedra that models free space
 */
class DecompUtil {
  public:
    ///Simple constructor
    DecompUtil(decimal_t r = 0);
    bool addSeed(const Vec3f& pt, Polyhedron& poly, Ellipsoid& ellipsoid);

    void addPoly(const Polyhedron& poly);

    void addEllipsoid(const Ellipsoid& e);

    void setObstacles(const vec_Vec3f& obs) { obs_ = obs; }

    ///Get polyhedra
    Polyhedra polyhedra();
    ///Get ellipsoids
    vec_Ellipsoid ellipsoids();
    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive& pr, decimal_t& dt);
    ///Check if a point is inside SFC
    bool isFree(const Vec3f& pt);

    /**
     * @brief Check if a primitive is inside given poly for a period
     * @param pr Input primitive
     * @param poly Input polyhedron
     * @param t1 Start time
     * @param dt Time of intersection
     */
    bool intersectPolyhedron(const Primitive& pr, const Polyhedron& poly, decimal_t t1, decimal_t& dt);
  private:
    ///Check if a point is inside the given polyhedron
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);

    ///Check if a primitive is inside the SFC from \f$t: t1\rightarrow t2\f$
    bool insideSFC(const Primitive& pr, decimal_t t1, decimal_t& dt);
 
    ///The SFC object
    Polyhedra polys_;

    decimal_t r_ = 0;

    vec_Vec3f ps_;
    vec_Ellipsoid es_;
    vec_Vec3f obs_;
};
#endif


