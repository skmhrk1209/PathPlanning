#include "path_planning.hpp"
#include "iostream.hpp"
#include <iostream>
#include <string>
#include <sstream>

#ifdef GRAPHICAL_DEBUG

#include <boost/process.hpp>

#endif // GRAPHICAL_DEBUG

template <typename Type>
decltype(auto) operator>>(std::istream& is, Point<Type>& vector)
{
	is >> vector.x() >> vector.y();

	return is;
}

template <typename Type>
decltype(auto) operator<<(std::ostream& os, const Point<Type>& vector)
{
	os << vector.x() << " " << vector.y();

	return os;
}

int main()
{
	int timeGiven, stepMax, width, length, vision;
	std::cin >> timeGiven >> stepMax >> width >> length >> vision;
	
	PathPlanner pathPlanner;
	pathPlanner.setup(width, length, vision);

	std::cout << 0 << std::endl;

#ifdef GRAPHICAL_DEBUG

	// create a child process for graphical debugging
	// geometryViewer is an applicaiton that visualizes geometries represented in WKT(Well-Known Text)
	const std::string path("GeometryViewer");
	boost::process::opstream pipe;
	boost::process::child child(path, boost::process::std_in < pipe);

	// register callback function that writes obstacle data in WKT(Well-Known Text) to pipe stream
	auto connection(pathPlanner.connect([&pipe](const auto& rings, const auto& voronoiDiagram, const auto& linestring)
	{
		pipe << "CLEAR" << std::endl;

		pipe << "COLOR 255, 0, 255, 255" << std::endl;

		for (const auto& ring : rings)
		{
			pipe << boost::geometry::wkt(ring) << std::endl;
		}

		pipe << "COLOR 0, 255, 255, 255" << std::endl;

		for (const auto& edge : voronoiDiagram.edges())
		{
			if (edge.is_secondary() || edge.is_infinite()) continue;

			Segment<Point<double>> segment(
				{ edge.vertex0()->x(), edge.vertex0()->y() },
				{ edge.vertex1()->x(), edge.vertex1()->y() });

			if (segment.first.hasNaN() || segment.second.hasNaN()) continue;

			pipe << boost::geometry::wkt(segment) << std::endl;
		}

		pipe << "COLOR 255, 255, 0, 255" << std::endl;

		pipe << boost::geometry::wkt(linestring) << std::endl;
	}));

#endif // GRAPHICAL_DEBUG

	Grid<int> grid(boost::extents[length + vision][width]);
	Point<int> lastPosition(0, -vision - 1);

	while (true)
	{
		int step, timeLeft;
		std::cin >> step >> timeLeft;
		
		Point<int> position, velocity;
		std::cin >> position >> velocity;

		Point<int> oppPosition, oppVelocity;
		std::cin >> oppPosition >> oppVelocity;

		std::cin.ignore();

		for (auto y(position.y() - vision); y <= position.y() + vision; ++y)
		{
			std::string line;
			std::getline(std::cin, line);

			if (y <= lastPosition.y() + vision) continue;

			std::istringstream sstream(line);

			for (auto x(0); x < width; ++x)
			{
				sstream >> grid[y][x];
			}
		}

		// update race information
		pathPlanner.update(position, velocity, grid);

		// seach an optimal solution using A* algorithm
		auto acceleration(pathPlanner.search());

		std::cout << acceleration << std::endl;

		lastPosition = std::move(position);
	}

	return 0;
}