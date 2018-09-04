#pragma once
#include <vector>
#include <queue>
#include <cmath>
#include <tuple>
#include <sstream>
#include <string>

struct Point {
	float x, y;

	Point(float X, float Y) : x(X), y(Y) {};

	//operator std::string () const {
	//	std::stringstream s;
	//	s << "(" << x << ", " << y << ")";
	//	return s.str();
	//}

	//operator const char*() const {
	//	std::stringstream s;
	//	s << "(" << x << ", " << y << ")";
	//	return s.str().c_str();
	//}

	bool operator==(const Point& other) {
		return this->x == other.x && this->y == other.y;
	}

	bool operator>(const Point& other) {
		if (this->x == other.x) {
			return this->y > other.y;
		} else {
			return this->x > other.x;
		}
	}

	bool operator<(const Point& other) {
		if (this->x == other.x) {
			return this->y < other.y;
		}
		else {
			return this->x < other.x;
		}
	}

	static float euclidean_distance(const Point& a, const Point& b) {
		return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
	}

	static float manhattan_distance(const Point& a, const Point& b) {
		return abs(a.x - b.x) + abs(a.y - b.y);
	}
};

std::ostream& operator<<(std::ostream &strm, const Point& pt) {
	std::stringstream s;
	s << "(" << pt.x << ", " << pt.y << ")";
	return strm << s.str();
}

template <>
struct std::hash<Point> {
	size_t operator()(const Point& obj) const {
		return (34 * obj.x * obj.x) + (obj.y * obj.y);
	}
};

template <typename T>
class SearchAlgorithm {
public:
	virtual std::tuple<float, std::vector<Edge<T>>> solve(const Graph<T>& graph, const Vertex<T>& start, const Vertex<T>& end) = 0;
};

template <typename T>
class UniformCostSearch : SearchAlgorithm<T> {
public:
	virtual std::tuple<float, std::vector<Edge<T>>> solve(const Graph<T>& graph, const Vertex<T>& start, const Vertex<T>& end);
};

template <typename T>
std::tuple<float, std::vector<Edge<T>>> UniformCostSearch<T>::solve(const Graph<T>& graph, const Vertex<T>& start, const Vertex<T>& end) {
	const std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*> adjList = *(graph.getAdjList());

	if (!adjList.count(start) || !adjList.count(end)) {
		fprintf(stderr, "Start or end not found in graph\n");
		exit(1);
	}

	if (start == end) {
		return std::make_tuple(0, std::vector<Edge<T>>());
	}
	
	std::unordered_set<Vertex<T>> visited;
	std::priority_queue<std::tuple<float, std::vector<Edge<T>>>, 
		std::vector<std::tuple<float, std::vector<Edge<T>>>>,
		std::greater<std::tuple<float, std::vector<Edge<T>>>>> pq;
	
	visited.insert(start);
	for (auto& edge : *adjList.at(start)) {
		pq.push(std::make_tuple<float, std::vector<Edge<T>>>(edge.getWeight(), { edge }));
	}
	while (!pq.empty()) {
		std::tuple<float, std::vector<Edge<T>>> root = pq.top();
		pq.pop();
		Edge<T> last = std::get<1>(root).back();
		Vertex<T> discVert = *last.getV();
		if (discVert == end) {
			return root;
		}
		if (visited.count(discVert)) {
			continue;
		} else {
			for (auto& edge : *adjList.at(discVert)) {
				if (!visited.count(*edge.getV())) {
					std::vector<Edge<T>> nextPath(std::get<1>(root));
					nextPath.push_back(edge);
					pq.push(std::make_tuple(std::get<0>(root) + edge.getWeight(), nextPath));
				}
			}
		}
		visited.insert(discVert);
	}
	
	fprintf(stderr, "End node not reachable from start\n");
	return std::make_tuple<float, std::vector<Edge<T>>>(-1, std::vector<Edge<T>>());
}

template <typename T>
class AStarSearch : SearchAlgorithm<T> {
	float (*_heuristic)(const T& curr, const T& end);
public:
	AStarSearch(float (*heuristic)(const T& curr, const T& end)) : _heuristic(heuristic) {};
	virtual std::tuple<float, std::vector<Edge<T>>> solve(const Graph<T>& graph, const Vertex<T>& start, const Vertex<T>& end);
};

template <typename T>
std::tuple<float, std::vector<Edge<T>>> AStarSearch<T>::solve(const Graph<T>& graph, const Vertex<T>& start, const Vertex<T>& end) {
	const std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*> adjList = *(graph.getAdjList());

	if (!adjList.count(start) || !adjList.count(end)) {
		fprintf(stderr, "Start or end not found in graph\n");
		exit(1);
	}

	if (start == end) {
		return std::make_tuple(0, std::vector<Edge<T>>());
	}

	std::unordered_set<Vertex<T>> visited;
	std::priority_queue<std::tuple<float, float, std::vector<Edge<T>>>,
		std::vector<std::tuple<float, float, std::vector<Edge<T>>>>,
		std::greater<std::tuple<float, float, std::vector<Edge<T>>>>> pq;

	visited.insert(start);
	for (auto& edge : *adjList.at(start)) {
		pq.push(std::make_tuple<float, float, std::vector<Edge<T>>>(edge.getWeight() + _heuristic(*(edge.getV()->getData()), *end.getData()), 
			edge.getWeight(), { edge }));
	}
	while (!pq.empty()) {
		std::tuple<float, float, std::vector<Edge<T>>> root = pq.top();
		pq.pop();
		Edge<T> last = std::get<2>(root).back();
		Vertex<T> discVert = *last.getV();
		if (discVert == end) {
			return std::make_tuple(std::get<1>(root), std::get<2>(root));
		}
		if (visited.count(discVert)) {
			continue;
		}
		else {
			for (auto& edge : *adjList.at(discVert)) {
				if (!visited.count(*edge.getV())) {
					std::vector<Edge<T>> nextPath(std::get<2>(root));
					nextPath.push_back(edge);
					pq.push(std::make_tuple(std::get<1>(root) + edge.getWeight() + _heuristic(*discVert.getData(), *end.getData()), std::get<1>(root) + edge.getWeight(), nextPath));
				}
			}
		}
		visited.insert(discVert);
	}

	fprintf(stderr, "End node not reachable from start\n");
	return std::make_tuple<float, std::vector<Edge<T>>>(-1, std::vector<Edge<T>>());
}