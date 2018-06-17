#pragma once

#define _SCL_SECURE_NO_WARNINGS

#include "node.hpp"
#include <vector>
#include <deque>
#include <utility>
#include <boost/multi_array.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/signals2/signal.hpp>
#include <Eigen/Dense>

#define GRAPHICAL_DEBUG

/**************************************************************

	2018/01/08 Sakuma Hiroki

	Samurai Coding Program

	Path Planning using voronoi diagram & A* algorithm

	https://github.com/skmhrk1209/AutonomousDriving

**************************************************************/

template <typename Type>
using Point = Eigen::Matrix<Type, 2, 1>;
template <typename Type>
using VoronoiDiagram = boost::polygon::voronoi_diagram<Type>;
template <typename Point>
using Ring = std::deque<Point>;
template <typename Point>
using Linestring = std::vector<Point>;
template <typename Point>
using Segment = std::pair<Point, Point>;
template <typename Type>
using Grid = boost::multi_array<Type, 2>;

BOOST_GEOMETRY_REGISTER_POINT_2D(Point<int>, int, boost::geometry::cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D(Point<float>, float, boost::geometry::cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D(Point<double>, double, boost::geometry::cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_RING_TEMPLATED(Ring)
BOOST_GEOMETRY_REGISTER_LINESTRING_TEMPLATED(Linestring)
BOOST_GEOMETRY_REGISTER_SEGMENT_TEMPLATIZED(Segment, first, second)

template <typename Type>
struct boost::polygon::geometry_concept<Point<Type>>
{
	using type = point_concept;
};

template <typename Type>
struct boost::polygon::point_traits<Point<Type>>
{
	using coordinate_type = Type;

	static inline Type get(const Point<Type>& point, orientation_2d orient)
	{
		return orient == HORIZONTAL ? point.x() : point.y();
	}
};

template <typename Type>
struct boost::polygon::geometry_concept<Segment<Point<Type>>>
{
	using type = segment_concept;
};

template <typename Type>
struct boost::polygon::segment_traits<Segment<Point<Type>>>
{
	using coordinate_type = Type;
	using point_type = Point<Type>;

	static inline Point<Type> get(const Segment<Point<Type>>& segment, direction_1d dir)
	{
		return dir.to_int() ? segment.first : segment.second;
	}
};

class PathPlanner
{
public:

	using State = std::pair<Point<int>, Point<int>>;
	using Value = std::pair<float, float>;

	void setup(int, int, int);
	void update(const Point<int>&, const Point<int>&, Grid<int>);

	Point<int> search();

	template <typename Function>
	boost::signals2::connection connect(const Function& function)
	{
		return mSignal.connect(function);
	}

private:

	int mWidth;
	int mLength;
	int mVision;

	Point<int> mPosition;
	Point<int> mVelocity;

	std::vector<Ring<Point<int>>> mRings;
	Linestring<Point<int>> mLinestring;

	std::vector<Segment<Point<int>>> mSegments;
	VoronoiDiagram<double> mVoronoiDiagram;

	Node<State, Value>::Ptr mRoot;
	Node<State, Value>::Ptr mGoal;

	boost::signals2::signal<void(
		const std::vector<Ring<Point<int>>>&,
		const VoronoiDiagram<double>&,
		const Linestring<Point<int>>&)> mSignal;
};