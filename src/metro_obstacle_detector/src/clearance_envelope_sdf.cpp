#include "metro_obstacle_detector/clearance_envelope_sdf.hpp"
#include <algorithm>
#include <cmath>

namespace metro_obstacle_detector
{

ClearanceEnvelopeSDF::ClearanceEnvelopeSDF(const ClearanceEnvelopeConfig & config)
: config_(config)
{
  update_polygon();
}

void ClearanceEnvelopeSDF::set_config(const ClearanceEnvelopeConfig & config)
{
  config_ = config;
  update_polygon();
}

void ClearanceEnvelopeSDF::update_polygon()
{
  // 14-vertex closed polygon for GOST 9238 Clearance Envelope (Габарит «М»)
  // Oriented in counter-clockwise order around the cross-section
  polygon_vertices_.clear();
  polygon_vertices_.reserve(14);

  // Right side: from bottom to top
  polygon_vertices_.push_back({config_.undercarriage_half_w, config_.rail_crown_margin});      // P0: bottom-right
  polygon_vertices_.push_back({config_.undercarriage_half_w, config_.undercarriage_height});    // P1: undercarriage top
  polygon_vertices_.push_back({config_.platform_half_w, config_.undercarriage_height});        // P2: platform bottom
  polygon_vertices_.push_back({config_.platform_half_w, config_.platform_height});            // P3: platform top
  polygon_vertices_.push_back({config_.waist_half_w, config_.platform_height});                // P4: waist bottom
  polygon_vertices_.push_back({config_.waist_half_w, config_.waist_height});                   // P5: waist top
  polygon_vertices_.push_back({config_.roof_half_w, config_.roof_height});                     // P6: roof top right

  // Left side: from top to bottom
  polygon_vertices_.push_back({-config_.roof_half_w, config_.roof_height});                    // P7: roof top left
  polygon_vertices_.push_back({-config_.waist_half_w, config_.waist_height});                  // P8: waist top left
  polygon_vertices_.push_back({-config_.waist_half_w, config_.platform_height});               // P9: waist bottom left
  polygon_vertices_.push_back({-config_.platform_half_w, config_.platform_height});           // P10: platform top left
  polygon_vertices_.push_back({-config_.platform_half_w, config_.undercarriage_height});       // P11: platform bottom left
  polygon_vertices_.push_back({-config_.undercarriage_half_w, config_.undercarriage_height});   // P12: undercarriage top left
  polygon_vertices_.push_back({-config_.undercarriage_half_w, config_.rail_crown_margin});     // P13: bottom-left
}

float ClearanceEnvelopeSDF::get_allowed_half_width(float v_elev) const
{
  if (v_elev < config_.rail_crown_margin) {
    return 0.0f; // Below rail crown
  }
  if (v_elev <= config_.undercarriage_height) {
    return config_.undercarriage_half_w;
  }
  if (v_elev <= config_.platform_height) {
    return config_.platform_half_w;
  }
  if (v_elev <= config_.waist_height) {
    return config_.waist_half_w;
  }
  if (v_elev <= config_.roof_height) {
    // Tapering to roof top
    const float span = std::max(0.001f, config_.roof_height - config_.waist_height);
    const float t = (v_elev - config_.waist_height) / span;
    return config_.waist_half_w + t * (config_.roof_half_w - config_.waist_half_w);
  }
  return 0.0f; // Above roof
}

float ClearanceEnvelopeSDF::compute_signed_distance(float u_lat, float v_elev) const
{
  // 1. Determine if point is inside the clearance polygon
  const bool inside = (v_elev >= config_.rail_crown_margin &&
                       v_elev <= config_.roof_height &&
                       std::abs(u_lat) <= get_allowed_half_width(v_elev));

  // 2. Compute exact minimum 2D Euclidean distance to the 14 polygon boundary segments
  float min_dist_sq = 1e12f;
  const size_t n = polygon_vertices_.size();

  for (size_t i = 0; i < n; ++i) {
    const auto & A = polygon_vertices_[i];
    const auto & B = polygon_vertices_[(i + 1) % n];

    const float ab_u = B.first - A.first;
    const float ab_v = B.second - A.second;
    const float ap_u = u_lat - A.first;
    const float ap_v = v_elev - A.second;

    const float ab_len_sq = ab_u * ab_u + ab_v * ab_v;
    float t = 0.0f;
    if (ab_len_sq > 1e-12f) {
      t = std::max(0.0f, std::min(1.0f, (ap_u * ab_u + ap_v * ab_v) / ab_len_sq));
    }

    const float q_u = A.first + t * ab_u;
    const float q_v = A.second + t * ab_v;

    const float diff_u = u_lat - q_u;
    const float diff_v = v_elev - q_v;
    const float d_sq = diff_u * diff_u + diff_v * diff_v;

    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
    }
  }

  const float min_dist = std::sqrt(min_dist_sq);

  // Exact 2D Euclidean distance: negative inside, positive outside
  return inside ? -min_dist : min_dist;
}

bool ClearanceEnvelopeSDF::is_inside_envelope(float u_lat, float v_elev, float margin) const
{
  return compute_signed_distance(u_lat, v_elev) <= margin;
}

bool ClearanceEnvelopeSDF::is_near_gauge_buffer(float u_lat, float v_elev) const
{
  const float sdf = compute_signed_distance(u_lat, v_elev);
  return (sdf > 0.0f && sdf <= config_.buffer_zone_margin);
}

std::vector<std::pair<float, float>> ClearanceEnvelopeSDF::get_polygon_contour() const
{
  std::vector<std::pair<float, float>> pts = polygon_vertices_;
  if (!pts.empty()) {
    pts.push_back(pts.front()); // Close contour
  }
  return pts;
}

} // namespace metro_obstacle_detector
