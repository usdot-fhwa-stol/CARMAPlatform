#include <inlanecruising_plugin/smoothing/BSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXf matrix_points(2, points.size());
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points, 2);
}
lanelet::BasicPoint2d BSpline::operator()(double t) const
{
  Eigen::VectorXf values = spline_(t);
  lanelet::BasicPoint2d pt = {(double)values.x(), (double)values.y()};
  return pt;
}

lanelet::BasicPoint2d  BSpline::first_deriv(double t) const {
  Eigen::Array2Xf v = spline_.derivatives(t, 1);
  lanelet::BasicPoint2d  output = {(double)v(2), (double)v(3)};
  return output;
}

lanelet::BasicPoint2d  BSpline::second_deriv(double t) const {
  Eigen::Array2Xf v = spline_.derivatives(t, 2);
  lanelet::BasicPoint2d  output = {(double)v(4), (double)v(5)};
  return output;
}

};  // namespace smoothing
};  // namespace inlanecruising_plugin