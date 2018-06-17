#include "path_planning.hpp"
#include <unordered_set>
#include <functional>
#include <algorithm>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp>

void PathPlanner::setup(int width, int length, int vision)
{
	mWidth = width;
	mLength = length;
	mVision = vision;
}

void PathPlanner::update(const Point<int>& position, const Point<int>& velocity, Grid<int> grid)
{
	auto inRange([](const auto& grid, const auto& point)
	{
		return
			point.x() >= 0 && point.x() < grid.shape()[1] &&
			point.y() >= 0 && point.y() < grid.shape()[0];
	});

	auto at([](auto& grid, const auto& point) ->decltype(auto)
	{
		return grid[point.y()][point.x()];
	});

	mPosition = position;
	mVelocity = velocity;

	mRings.clear();
	mSegments.clear();

	enum { NotObstacle, NotVisited, Visited };

	for (auto y(0); y <= mPosition.y() + mVision; ++y)
	{
		for (auto x(0); x < mWidth; ++x)
		{
			if (grid[y][x] != NotVisited) continue;

			mRings.emplace_back();
			decltype(auto) mRing(mRings.back());

			Point<int> position(x, y);
			Point<float> direction(0, 1);

			Eigen::Rotation2D<float> rotationPositive(+EIGEN_PI / 2);
			Eigen::Rotation2D<float> rotationNegative(-EIGEN_PI / 4);

			mRing.emplace_back(position);
			at(grid, position) = Visited;

			// search the boundary of an obstacle and push the point to the ring geometry
			while (true)
			{
				direction = rotationPositive * direction;
				direction = direction.array().round();

				for (auto i(0); i < 8; ++i)
				{
					Point<int> nextPosition(position + direction.cast<int>());

					if (inRange(grid, nextPosition) && at(grid, nextPosition) != NotObstacle)
					{
						mRing.emplace_back(nextPosition);
						
						mSegments.emplace_back(position, nextPosition);

						at(grid, nextPosition) = Visited;

						position = std::move(nextPosition);

						break;
					}

					direction = rotationNegative * direction;
					direction = direction.array().round();
				}

				if (position == mRing.front()) break;
			}

			boost::geometry::model::box<Point<int>> boundingBox;
			boost::geometry::envelope(mRing, boundingBox);

			for (auto yy(boundingBox.min_corner().y()); yy <= boundingBox.max_corner().y(); ++yy)
			{
				for (auto xx(boundingBox.min_corner().x()); xx <= boundingBox.max_corner().x(); ++xx)
				{
					if (boost::geometry::within(Point<int>(xx, yy), mRing))
					{
						grid[yy][xx] = Visited;
					}
				}
			}
		}
	}

	auto maxLength(std::numeric_limits<int>::max());

	mRings.emplace_back<Ring<Point<int>>>({ 
		{ -1, maxLength }, { -1, -1 }, { mWidth, -1 }, { mWidth, maxLength }, 
		{ mWidth, maxLength }, { mWidth, -1 } , { -1, -1 }, { -1, maxLength }});

	mSegments.emplace_back<Point<int>, Point<int>>({ -1, maxLength }, { -1, -1 });
	mSegments.emplace_back<Point<int>, Point<int>>({ -1, -1 }, { mWidth, -1 });
	mSegments.emplace_back<Point<int>, Point<int>>({ mWidth, -1 }, { mWidth, maxLength });
	
	std::sort(mSegments.begin(), mSegments.end(), [](const auto& segment1, const auto& segment2)
	{
		return std::less<std::pair<
			std::pair<typename decltype(segment1.first)::Scalar, typename decltype(segment1.first)::Scalar>, 
			std::pair<typename decltype(segment1.second)::Scalar, typename decltype(segment1.second)::Scalar>>>()(
				{ { segment1.first.x(), segment1.first.y() }, { segment1.second.x(), segment1.second.y() } },
				{ { segment2.first.x(), segment2.first.y() }, { segment2.second.x(), segment2.second.y() } });
	});

	mSegments.erase(std::unique(mSegments.begin(), mSegments.end()), mSegments.end());
	
	mVoronoiDiagram.clear();
	boost::polygon::construct_voronoi(mSegments.begin(), mSegments.end(), &mVoronoiDiagram);

#ifdef GRAPHICAL_DEBUG

	mLinestring.emplace_back(mPosition);

	mSignal(mRings, mVoronoiDiagram, mLinestring);

#endif // GRAPHICAL_DEBUG
}

Point<int> PathPlanner::search()
{
	// collision detection
	auto collided([this](const auto& point1, const auto& point2)
	{
		return std::find_if(mRings.begin(), mRings.end(), [&](const auto& mRing)
		{
			return boost::geometry::intersects(Segment<Point<int>>(point1, point2), mRing);

		}) != mRings.end();
	});

	auto duration([this](const auto& position, const auto& velocity)
	{
		auto equation([](auto a, auto b, auto c)
		{
			return (-b + sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
		});

		return std::max(0.0, equation(1.0, 2.0 * velocity.y() + 1.0, -2.0 * (mPosition.y() + mVision - position.y())));
	});

	auto potential([this](const auto& position, const auto& velocity)
	{
		auto maxDistance(std::numeric_limits<float>::max());

		auto distanceToEdge(maxDistance);

		for (const auto& edge : mVoronoiDiagram.edges())
		{
			if (edge.is_secondary() || edge.is_infinite()) continue;

			Segment<Point<double>> segment(
				{ edge.vertex0()->x(), edge.vertex0()->y() },
				{ edge.vertex1()->x(), edge.vertex1()->y() });

			if (segment.first.hasNaN() || segment.second.hasNaN()) continue;

			distanceToEdge = std::min<decltype(distanceToEdge)>(
				distanceToEdge, boost::geometry::distance(position, segment));
		}

		auto distanceToObstacle(maxDistance);

		for (const auto& mRing : mRings)
		{
			distanceToObstacle = std::min<decltype(distanceToObstacle)>(
				distanceToObstacle, boost::geometry::distance(position, mRing));
		}

		return 500.0 / (500.0 + distanceToObstacle) * distanceToEdge / (distanceToObstacle + distanceToEdge) * 
			(distanceToObstacle - maxDistance) * (distanceToObstacle - maxDistance) / maxDistance / maxDistance;
	});

	// heuristic function
	auto heuristic([&duration, &potential](const auto& position, const auto& velocity)
	{
		return 0.6 * duration(position, velocity) + 0.4 * potential(position, velocity);
	});

	auto finished([this](const auto& state)
	{
		return state.first.y() >= mPosition.y() + mVision;
	});

	// comparator for priority queue
	auto greater([](const auto& node1, const auto& node2)
	{
		return 
			node1->value().first + node1->value().second > 
			node2->value().first + node2->value().second;
	});

	// hasher for hash map
	auto hash([](const auto& node)
	{
		return boost::hash<std::pair<
			std::pair<typename decltype(node->state().first)::Scalar, typename decltype(node->state().first)::Scalar>, 
			std::pair<typename decltype(node->state().second)::Scalar, typename decltype(node->state().second)::Scalar>>>()(
				{ { node->state().first.x(), node->state().first.y() }, { node->state().second.x(), node->state().second.y() }});
	});

	// key equal for hash map
	auto equal([](const auto& node1, const auto& node2)
	{
		return node1->state() == node2->state();
	});

	// open list
	boost::heap::fibonacci_heap<Node<State, Value>::Ptr, boost::heap::compare<decltype(greater)>> openList(greater);
	// closed list
	std::unordered_set<Node<State, Value>::Ptr, decltype(hash), decltype(equal)> closedList(0, hash, equal);

	mRoot = Node<State, Value>::create<State, Value>({ mPosition, mVelocity }, { 0.0, heuristic(mPosition, mVelocity) });

	openList.push(mRoot);

	std::vector<Point<int>> accelerations;

	for (auto y(-1); y <= 1; ++y)
	{
		for (auto x(-1); x <= 1; ++x)
		{
			accelerations.emplace_back(x, y);
		}
	}

	// A* algorithm steps
	while (!openList.empty())
	{
		auto parent(openList.top());

		openList.pop();
		closedList.insert(parent);

		if (!finished(parent->state()))
		{
			for (const auto& acceleration : accelerations)
			{
				Point<int> velocity(parent->state().second + acceleration);
				Point<int> position(parent->state().first + velocity);

				if (collided(parent->state().first, position)) continue;

				Value value(parent->value().first + 1.0, heuristic(position, velocity));
				State state(std::move(position), std::move(velocity));

				auto child(Node<State, Value>::create(parent, state, value));

				auto it(closedList.find(child));

				if (it == closedList.end())
				{
					auto jt(find_if(openList.begin(), openList.end(), std::bind(equal, std::cref(child), std::placeholders::_1)));

					if (jt == openList.end())
					{
						openList.push(child);
						parent->children().emplace_back(child);
					}
					else
					{
						if ((*jt)->value().first + (*jt)->value().second >
							child->value().first + child->value().second)
						{
							openList.erase(decltype(openList)::s_handle_from_iterator(jt));
							openList.push(child);
							parent->children().emplace_back(child);
						}
					}
				}
				else
				{
					if ((*it)->value().first + (*it)->value().second >
						child->value().first + child->value().second)
					{
						closedList.erase(it);
						openList.push(child);
						parent->children().emplace_back(child);
					}
				}
			}
		}
		// an optimal solution was found
		else
		{
			mGoal = parent;

			auto node(mGoal);

			while (auto parent = node->parent().lock())
			{
				if (parent == mRoot)
				{
					return node->state().second - parent->state().second;
				}

				node = parent;
			}
		}
	}

	// any optimal solutions were not found
	return -mVelocity.array().sign();
}
