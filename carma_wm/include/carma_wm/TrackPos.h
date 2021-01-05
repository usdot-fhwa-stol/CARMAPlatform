#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

namespace carma_wm
{
/*! \brief Position in a track based coordinate system where the axis are downtrack and crosstrack.
 *         Positive crosstrack is to the left of the reference line
 *
 * The position of a point relative to a reference line. The perpendicular distance from the point to the reference line
 * is the crosstrack The distance to the intersection of the perpendicular from the start of the line is the downtrack
 * distance If an x,y point would be located to the right of the reference line then the crosstrack is positive. It is
 * negative if on the left side of the line. NOTE: The positive/negative crosstrack side is flipped relative to a
 * lanelet::ArcCoordinate Most utility functions will assume that the downtrack/crosstrack values are specified in
 * meters
 */
class TrackPos
{
public:
  double downtrack = 0;
  double crosstrack = 0;

  /*! \brief Constructor
   *
   * \param down_track The downtrack distance
   * \param cross_track The crosstrack distance
   */
  TrackPos(double down_track, double cross_track) : downtrack(down_track), crosstrack(cross_track)
  {
  }
  /*! \brief Constructor from lanelet ArcCoordinates
   *         which are converted to TrackPos where downtrack = ArcCoordinates.length and crosstrack =
   * -ArcCoordinates.distance
   *
   * \param arc_coord ArcCoordinate to copy
   */
  TrackPos(const lanelet::ArcCoordinates& arc_coord) : downtrack(arc_coord.length), crosstrack(-arc_coord.distance)
  {
  }

  /*! \brief Returns a lanelet ArcCoordinate built from this TrackPos
   *        where downtrack = ArcCoordinates.length and crosstrack = -ArcCoordinates.distance
   *
   * \return lanelet::ArcCoordinates with values copied from this object
   */
  inline lanelet::ArcCoordinates toArcCoordinates()
  {
    return lanelet::ArcCoordinates{ downtrack, -crosstrack };
  }

  // Overload == and != operators
  bool operator==(const TrackPos& other) const
  {
    return this == &other || (this->downtrack == other.downtrack && this->crosstrack == other.crosstrack);
  }

  bool operator!=(const TrackPos& other) const
  {
    return !(*this == other);
  }
};
}  // namespace carma