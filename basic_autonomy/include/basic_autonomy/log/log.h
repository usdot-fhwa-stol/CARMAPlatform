#include <sstream>
#include <basic_autonomy/basic_autonomy.h>

namespace basic_autonomy
{
namespace log
{
/**
 * \brief Helper function to convert a lanelet::BasicPoint2d to a string
 */ 
std::string basicPointToStream(lanelet::BasicPoint2d point)
{
  std::ostringstream out;
  out << point.x() << ", " << point.y();
  return out.str();
}
/**
 * \brief Helper function to convert a PointSpeedPair to a string
 */ 
std::string pointSpeedPairToStream(waypoint_generation::PointSpeedPair point)
{
  std::ostringstream out;
  out << "Point: " << basicPointToStream(point.point) << " Speed: " << point.speed;
  return out.str();
}

/**
 * \brief Print a ROS_DEBUG_STREAM for each value in values where the printed value is a string returned by func
 */ 
template <class T>
void printDebugPerLine(const std::vector<T>& values, std::function<std::string(T)> func)
{
  for (const auto& value : values)
  {
    ROS_DEBUG_STREAM(func(value));
  }
}

/**
 * \brief Print a ROS_DEBUG_STREAM for each value in values where the printed value is a string returned by free_func
 */ 
template <class T>
void printDebugPerLine(const std::vector<T>& values, std::string (*free_func)(T))
{
  auto function = static_cast<std::function<std::string(T)>>(free_func);
  printDebugPerLine(values, function);
}

/**
 * \brief Print a ROS_DEBUG_STREAM for each value in values where the printed value is << prefix << value
 */ 
void printDoublesPerLineWithPrefix(const std::string& prefix, const std::vector<double>& values)
{
  for (const auto& value : values)
  {
    ROS_DEBUG_STREAM(prefix << value);
  }
}

}  // namespace log
}  // namespace basic_autonomy