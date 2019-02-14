/*
Copyright (C) 2019 Michael Fogleman

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "dlaf.h"
// stl
#include <chrono>
#include <random>
// boost
#include <boost/function_output_iterator.hpp>
#include <boost/geometry/geometry.hpp>

namespace dlaf {

  using namespace ospcommon;

  // number of dimensions (must be 2 or 3)
  static constexpr int D = 3;

  // default parameters (documented below)
  static constexpr double DefaultParticleSpacing    = 1;
  static constexpr double DefaultAttractionDistance = 3;
  static constexpr double DefaultMinMoveDistance    = 1;
  static constexpr int DefaultStubbornness          = 0;
  static constexpr double DefaultStickiness         = 1;

  // boost is used for its spatial index
  using BoostPoint =
      boost::geometry::model::point<double, D, boost::geometry::cs::cartesian>;

  using IndexValue = std::pair<BoostPoint, int>;

  using Index =
      boost::geometry::index::rtree<IndexValue,
                                    boost::geometry::index::linear<4>>;

  static inline BoostPoint ToBoost(const vec3f &v)
  {
    return BoostPoint(v.x, v.y, v.z);
  }

  // Random returns a uniformly distributed random number between lo and hi
  double Random(const double lo = 0, const double hi = 1)
  {
    static thread_local std::mt19937 gen(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(gen);
  }

  // RandomInUnitSphere returns a random, uniformly distributed point inside the
  // unit sphere (radius = 1)
  vec3f RandomInUnitSphere()
  {
    while (true) {
      const vec3f p =
          vec3f(Random(-1, 1), Random(-1, 1), D == 2 ? 0 : Random(-1, 1));
      if (length(p) * length(p) < 1) {
        return p;
      }
    }
  }

  // Model holds all of the particles and defines their behavior.
  class Model
  {
   public:
    Model()
        : m_ParticleSpacing(DefaultParticleSpacing),
          m_AttractionDistance(DefaultAttractionDistance),
          m_MinMoveDistance(DefaultMinMoveDistance),
          m_Stubbornness(DefaultStubbornness),
          m_Stickiness(DefaultStickiness),
          m_BoundingRadius(0)
    {
    }

    void SetParticleSpacing(const double a)
    {
      m_ParticleSpacing = a;
    }

    void SetAttractionDistance(const double a)
    {
      m_AttractionDistance = a;
    }

    void SetMinMoveDistance(const double a)
    {
      m_MinMoveDistance = a;
    }

    void SetStubbornness(const int a)
    {
      m_Stubbornness = a;
    }

    void SetStickiness(const double a)
    {
      m_Stickiness = a;
    }

    // Add adds a new particle with the specified parent particle
    void Add(const vec3f &p, const int parent = -1)
    {
      const int id = m_Points.size();
      m_Index.insert(std::make_pair(ToBoost(p), id));
      m_Points.push_back(p);
      m_JoinAttempts.push_back(0);
      m_BoundingRadius =
          std::max(m_BoundingRadius, length(p) + m_AttractionDistance);
    }

    // Nearest returns the index of the particle nearest the specified point
    int Nearest(const vec3f &point) const
    {
      int result = -1;
      m_Index.query(
          boost::geometry::index::nearest(ToBoost(point), 1),
          boost::make_function_output_iterator(
              [&result](const auto &value) { result = value.second; }));
      return result;
    }

    // RandomStartingPosition returns a random point to start a new particle
    vec3f RandomStartingPosition() const
    {
      const double d = m_BoundingRadius;
      return normalize(RandomInUnitSphere()) * d;
    }

    // ShouldReset returns true if the particle has gone too far away and
    // should be reset to a new random starting position
    bool ShouldReset(const vec3f &p) const
    {
      return length(p) > m_BoundingRadius * 2;
    }

    // ShouldJoin returns true if the point should attach to the specified
    // parent particle. This is only called when the point is already within
    // the required attraction distance.
    bool ShouldJoin(const vec3f &p, const int parent)
    {
      m_JoinAttempts[parent]++;
      if (m_JoinAttempts[parent] < m_Stubbornness) {
        return false;
      }
      return Random() <= m_Stickiness;
    }

    // PlaceParticle computes the final placement of the particle.
    vec3f PlaceParticle(const vec3f &p, const int parent) const
    {
      return lerp(m_ParticleSpacing, m_Points[parent], p);
    }

    // Motionvec3f returns a vector specifying the direction that the
    // particle should move for one iteration. The distance that it will move
    // is determined by the algorithm.
    vec3f Motionvec3f(const vec3f &p) const
    {
      return RandomInUnitSphere();
    }

    // AddParticle diffuses one new particle and adds it to the model
    void AddParticle()
    {
      // compute particle starting location
      vec3f p = RandomStartingPosition();

      // do the random walk
      while (true) {
        // get distance to nearest other particle
        const int parent = Nearest(p);
        const double d   = length(p - m_Points[parent]);

        // check if close enough to join
        if (d < m_AttractionDistance) {
          if (!ShouldJoin(p, parent)) {
            // push particle away a bit
            p = lerp(
                m_AttractionDistance + m_MinMoveDistance, m_Points[parent], p);
            continue;
          }

          // adjust particle position in relation to its parent
          p = PlaceParticle(p, parent);

          // add the point
          Add(p, parent);
          return;
        }

        // move randomly
        const double m = std::max(m_MinMoveDistance, d - m_AttractionDistance);
        p += normalize(Motionvec3f(p)) * m;

        // check if particle is too far away, reset if so
        if (ShouldReset(p)) {
          p = RandomStartingPosition();
        }
      }
    }

    containers::AlignedVector<vec3f> consumeFinalPoints()
    {
      return std::move(m_Points);
    }

   private:
    // m_ParticleSpacing defines the distance between particles that are
    // joined together
    double m_ParticleSpacing;

    // m_AttractionDistance defines how close together particles must be in
    // order to join together
    double m_AttractionDistance;

    // m_MinMoveDistance defines the minimum distance that a particle will move
    // during its random walk
    double m_MinMoveDistance;

    // m_Stubbornness defines how many interactions must occur before a
    // particle will allow another particle to join to it.
    int m_Stubbornness;

    // m_Stickiness defines the probability that a particle will allow another
    // particle to join to it.
    double m_Stickiness;

    // m_BoundingRadius defines the radius of the bounding sphere that bounds
    // all of the particles
    double m_BoundingRadius;

    // m_Points stores the final particle positions
    containers::AlignedVector<vec3f> m_Points;

    // m_JoinAttempts tracks how many times other particles have attempted to
    // join with each finalized particle
    containers::AlignedVector<int> m_JoinAttempts;

    // m_Index is the spatial index used to accelerate nearest neighbor queries
    Index m_Index;
  };

  containers::AlignedVector<vec3f> compute_points()
  {
    // create the model
    Model model;

    // add seed point(s)
#if 1
    model.Add(vec3f(0.f));
#else
    {
      const int n    = 3600;
      const double r = 1000;
      for (int i = 0; i < n; i++) {
        const double t = (double)i / n;
        const double a = t * 2 * M_PI;
        const double x = std::cos(a) * r;
        const double y = std::sin(a) * r;
        model.Add(vec3f(x, y, 0));
      }
    }
#endif

    // run diffusion-limited aggregation
    for (int i = 0; i < 100000; i++)
      model.AddParticle();

    return std::move(model.consumeFinalPoints());
  }

}  // namespace dlaf
