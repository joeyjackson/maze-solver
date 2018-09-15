#include "MazeSolver.h"
#define BLACK 0
#define WHITE ((255 << 16) + (255 << 8) + 255)
#define GREEN (255 << 8)
#define RED 255

MazeSolver::MazeSolver(const cv::Mat& image, std::shared_ptr<SearchAlgorithm<Point>> algorithm) : m_algo(algorithm) {
	this->m_graph = create_graph_from_image(image, &this->m_start, &this->m_end);
}

MazeSolver::MazeSolver(const Graph<Point>& graph, std::shared_ptr<SearchAlgorithm<Point>> algorithm) : m_algo(algorithm), m_graph(graph) {}

MazeSolver::MazeSolver(const Graph<Point>& graph, const Vertex<Point>& start, const Vertex<Point>& end,
	std::shared_ptr<SearchAlgorithm<Point>> algorithm) : m_algo(algorithm), m_graph(graph), m_start(start), m_end(end) {}

static Graph<Point> create_graph_from_image(const cv::Mat& image, Vertex<Point>* start_dst, Vertex<Point>* end_dst) {
	std::unordered_set<Vertex<Point>> vertices;
	std::unordered_set<Edge<Point>> edges;
	std::unordered_map<std::tuple<int, int>, std::shared_ptr<Vertex<Point>>> v_ptrs;
	for (int r = 0; r < image.rows; ++r) {
		for (int c = 0; c < image.cols; ++c) {
			cv::Vec3b pixel = image.at<cv::Vec3b>(r, c);
			if (!isWall(image, r, c)) {
				int color = (pixel[0] << 16) + (pixel[1] << 8) + pixel[2];
				if (color == GREEN) {
					*start_dst = Vertex<Point>(std::make_shared<Point>(c, r));
				}
				else if (color == RED) {
					*end_dst = Vertex<Point>(std::make_shared<Point>(c, r));
				}
				else if (color == WHITE) {

				}
				vertices.insert(Vertex<Point>(std::make_shared<Point>(c, r)));
				v_ptrs.emplace(std::make_tuple(c, r), std::make_shared<Vertex<Point>>(std::make_shared<Point>(c, r)));
				if (!isWall(image, r - 1, c)) {
					edges.insert(Edge<Point>(v_ptrs[std::make_tuple(c, r)], v_ptrs[std::make_tuple(c, r - 1)], 1));
				}
				if (!isWall(image, r, c - 1)) {
					edges.insert(Edge<Point>(v_ptrs[std::make_tuple(c, r)], v_ptrs[std::make_tuple(c - 1, r)], 1));
				}
			}
		}
	}
	return Graph<Point>(vertices, edges, true);
}

static inline bool isWall(const cv::Mat& image, int row, int col) {
	if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) {
		return true;
	}
	cv::Vec3b pixel = image.at<cv::Vec3b>(row, col);
	if (pixel[0] != 255 && pixel[1] != 255 && pixel[2] != 255) {
		return true;
	}
	return false;
}

std::tuple<float, std::vector<Edge<Point>>> MazeSolver::solve() const {
	return m_algo->solve(this->m_graph, this->m_start, this->m_end);
}

std::tuple<float, std::vector<Edge<Point>>> MazeSolver::solve(const Vertex<Point>& start, const Vertex<Point>& end) const {
	return m_algo->solve(this->m_graph, start, end);
}

const Graph<Point>* MazeSolver::getGraph() const {
	return &m_graph;
}

Graph<Point>* MazeSolver::getGraph() {
	return &m_graph;
}

void MazeSolver::setAlgorithm(const std::shared_ptr<SearchAlgorithm<Point>>& algorithm) {
	this->m_algo = algorithm;
}

void MazeSolver::setStart(const Vertex<Point>& start) {
	this->m_start = start;
}

void MazeSolver::setEnd(const Vertex<Point>& end) {
	this->m_end = end;
}