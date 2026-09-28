#pragma once

#include "metro_obstacle_detector/types.hpp"
#include <vector>
#include <utility>

namespace metro_obstacle_detector
{

/**
 * @brief GOST 9238 Clearance Envelope configuration (Габарит «М» для метрополитена).
 *
 * Defines the cross-sectional kinematic gauge dimensions relative to top-of-rail (УГР):
 * - Rail crown margin: v_min = 0.12 m
 * - Undercarriage: v in [0.12, 0.45] m -> allowed half-width w = 1.05 m
 * - Station platform level: v in (0.45, 1.25] m -> allowed half-width w = 1.30 m
 * - Waist: v in (1.25, 2.45] m -> allowed half-width w = 1.35 m
 * - Roof: v in (2.45, 2.80] m -> linear tapering to w = 1.10 m
 */
struct ClearanceEnvelopeConfig
{
  float rail_crown_margin{0.12f};      // Minimum height above rail crown (m), v_min = 0.12m
  float undercarriage_height{0.45f};   // Undercarriage boundary height (m), v <= 0.45m
  float undercarriage_half_w{1.05f};   // Undercarriage allowed half-width (m), w = 1.05m
  float platform_height{1.25f};        // Station platform level boundary (m), v <= 1.25m
  float platform_half_w{1.30f};        // Platform level allowed half-width (m), w = 1.30m
  float waist_height{2.45f};           // Waist boundary height (m), v <= 2.45m
  float waist_half_w{1.35f};           // Waist allowed half-width (m), w = 1.35m
  float roof_height{2.80f};            // Maximum roof height (m), v <= 2.80m
  float roof_half_w{1.10f};            // Roof arch top allowed half-width (m), tapering to w = 1.10m
  float buffer_zone_margin{0.30f};     // Near-gauge buffer zone (m)
};

class ClearanceEnvelopeSDF
{
public:
  explicit ClearanceEnvelopeSDF(const ClearanceEnvelopeConfig & config = ClearanceEnvelopeConfig{});

  void set_config(const ClearanceEnvelopeConfig & config);
  const ClearanceEnvelopeConfig & get_config() const { return config_; }

  /**
   * @brief Compute exact 2D Euclidean signed distance d(u, v) to the GOST 9238 polygon boundary.
   *
   * @param u_lat Lateral displacement from track center (m).
   * @param v_elev Vertical elevation above rail head (УГР) (m).
   * @return Negative minimal Euclidean distance when inside clearance;
   *         Positive minimal Euclidean distance when outside clearance;
   *         0.0 on the exact polygon boundary.
   */
  float compute_signed_distance(float u_lat, float v_elev) const;

  /**
   * @brief Check if point (u, v) is inside the kinematic envelope (with optional margin).
   */
  bool is_inside_envelope(float u_lat, float v_elev, float margin = 0.0f) const;

  /**
   * @brief Check if point (u, v) is in the near-gauge advisory buffer zone (0 < sdf <= margin).
   */
  bool is_near_gauge_buffer(float u_lat, float v_elev) const;

  /**
   * @brief Get allowed half-width at given elevation v above rail crown.
   */
  float get_allowed_half_width(float v_elev) const;

  /**
   * @brief Generate 2D polygon vertices in counter-clockwise order for visualization / rendering.
   */
  std::vector<std::pair<float, float>> get_polygon_contour() const;

private:
  void update_polygon();

  ClearanceEnvelopeConfig config_;
  std::vector<std::pair<float, float>> polygon_vertices_;
};

} // namespace metro_obstacle_detector
