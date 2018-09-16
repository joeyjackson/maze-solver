#pragma once

#include "SearchAlgorithms.h"
#include "Graph.h"
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

class MazeSolver {
private:
	std::shared_ptr<SearchAlgorithm<Point>> m_algo;
	Graph<Point> m_graph;
	Vertex<Point> m_start;
	Vertex<Point> m_end;
public:
	MazeSolver(const cv::Mat& image, std::shared_ptr<SearchAlgorithm<Point>> algorithm = std::make_shared<AStarSearch<Point>>(Point::euclidean_distance));
	MazeSolver(const Graph<Point>& graph, std::shared_ptr<SearchAlgorithm<Point>> algorithm = std::make_shared<AStarSearch<Point>>(Point::euclidean_distance));
	
	MazeSolver(const Graph<Point>& graph, const Vertex<Point>& start, const Vertex<Point>& end, 
		std::shared_ptr<SearchAlgorithm<Point>> algorithm = std::make_shared<AStarSearch<Point>>(Point::euclidean_distance));

	std::tuple<float, std::vector<Edge<Point>>> solve() const;
	std::tuple<float, std::vector<Edge<Point>>> solve(const Vertex<Point>& start, const Vertex<Point>& end) const;

	const Graph<Point>* getGraph() const;
	Graph<Point>* getGraph();
	void setAlgorithm(const std::shared_ptr<SearchAlgorithm<Point>>& algorithm);
	void setStart(const Vertex<Point>& start);
	void setEnd(const Vertex<Point>& end);
};

static inline bool isWall(const cv::Mat& image, int row, int col);
static Graph<Point> create_graph_from_image(const cv::Mat& image, Vertex<Point>* start_dst, Vertex<Point>* end_dst);