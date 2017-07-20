#include <collision_checking/decomp_util.h>

DecompUtil::DecompUtil(decimal_t r): r_(r) {}

Polyhedra DecompUtil::polyhedra() {
  return polys_;
}

vec_Ellipsoid DecompUtil::ellipsoids() {
  return es_;
}

bool DecompUtil::isFree(const Primitive& pr, decimal_t& dt) {
  return insideSFC(pr, 0, dt);
}

bool DecompUtil::isFree(const Vec3f& pt) {
  // Check if a point is inside free space
  // note that it can be inside unknown space as it is inside the free space of at least one polyhedron
  for(const auto& Vs: polys_) {
    if(insidePolyhedron(pt, Vs)) 
      return true;
  }
  return false;
}

bool DecompUtil::insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs) {
  for (const auto& v : Vs) {
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    if (a.dot(pt) - b > 1e-3)
      return false;
  }
  return true;
}

bool DecompUtil::intersectPolyhedron(const Primitive& pr, const Polyhedron& poly, decimal_t t1, decimal_t& dt) {
  const vec_E<Vec6f> cs = pr.coeffs();
  decimal_t min_t = std::numeric_limits<decimal_t>::max();
 
  decimal_t t2 = pr.t();
  for(const auto& v: poly) {
    Vec3f n = v.n;
    decimal_t a1 = n(0);
    decimal_t a2 = n(1);
    decimal_t a3 = n(2);

    decimal_t a = (a1*cs[0](1)+a2*cs[1](1)+a3*cs[2](1))/24;
    decimal_t b = (a1*cs[0](2)+a2*cs[1](2)+a3*cs[2](2))/6;
    decimal_t c = (a1*cs[0](3)+a2*cs[1](3)+a3*cs[2](3))/2;
    decimal_t d = (a1*cs[0](4)+a2*cs[1](4)+a3*cs[2](4));
    decimal_t e = a1*cs[0](5)+a2*cs[1](5)+a3*cs[2](5)-n.dot(v.p);

    std::vector<decimal_t> ts = solve(a, b, c, d, e);
    //printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
    for(const auto& it: ts) {
      if(it > t1 && it < t2)
      {
        if(it < min_t)
          min_t = it;
      }
    }
  }
  if(min_t < t2) {
    dt = min_t;
    return true;
  }
  else {
    dt = t2;
    return false;
  }
}

bool DecompUtil::insideSFC(const Primitive& pr, decimal_t t1, decimal_t& dt) {
  dt = t1;
  Waypoint p0 = pr.evaluate(t1 + 1e-2);

  //Polyhedra related_polys = polys_;
  Polyhedra related_polys;
  for(int i = 0; i < polys_.size(); i++) {
    if((ps_[i] - p0.pos).norm() < 4*1.4*r_)
      related_polys.push_back(polys_[i]);
  }

  Polyhedra interest_polys;
  for(int i = 0; i < (int) related_polys.size(); i++) {
    if(insidePolyhedron(p0.pos, related_polys[i]))
      interest_polys.push_back(related_polys[i]);
  }

  if(interest_polys.empty())
    return false;
  
  decimal_t max_dt = dt;
  for(const auto& poly: interest_polys) {
    decimal_t tmp_dt;
    if(intersectPolyhedron(pr, poly, t1, tmp_dt)) {
      if(tmp_dt > max_dt)
        max_dt = tmp_dt;
    }
    //if find one polyhedron, no intersection, return true and set dt
    else {
      dt = tmp_dt;
      return true;
    }
  }

  dt = max_dt;
  return insideSFC(pr, max_dt, dt);
}

bool DecompUtil::addSeed(const Vec3f& pt, Polyhedron& poly, Ellipsoid& ellipsoid) {
  SeedDecomp decomp(pt);
  decomp.set_virtual_dim(4*r_, 4*r_, 0.1);
  decomp.set_obstacles(obs_);
  decomp.dilate(r_);
  if(decomp.radius() < r_)
    return false;
  else {
    decomp.shrink(r_);
    poly = decomp.polyhedron();
    ellipsoid = decomp.ellipsoid();
    return true;
  }
}

void DecompUtil::addEllipsoid(const Ellipsoid& e) {
  es_.push_back(e);
  ps_.push_back(e.second);
}

void DecompUtil::addPoly(const Polyhedron& poly) {
  polys_.push_back(poly);
}
