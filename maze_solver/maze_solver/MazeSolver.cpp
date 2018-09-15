#include "MazeSolver.h"
#define BLACK 0
#define WHITE ((255 << 16) + (255 << 8) + 255)
#define GREEN (255 << 8)
#define RED 255

MazeSolver::MazeSolver(const cv::Mat& image, std::shared_ptr<SearchAlgorithm<Point>> algorithm) : m_algo(algorithm) {
	this->m_graph = create_graph_from_image(image);
}

MazeSolver::MazeSolver(const Graph<Point>& graph, std::shared_ptr<SearchAlgorithm<Point>> algorithm) : m_algo(algorithm), m_graph(graph) {}

static Graph<Point> create_graph_from_image(const cv::Mat& image) {
	std::unordered_set<Vertex<Point>> vertices;
	std::unordered_set<Edge<Point>> edges;
	std::unordered_map<std::tuple<int, int>, std::shared_ptr<Vertex<Point>>> v_ptrs;
	for (int r = 0; r < image.rows; ++r) {
		for (int c = 0; c < image.cols; ++c) {
			cv::Vec3b pixel = image.at<cv::Vec3b>(r, c);
			if (!isWall(image, r, c)) {
				int color = (pixel[0] << 16) + (pixel[1] << 8) + pixel[2];
				if (color == GREEN) {

				}
				else if (color == RED) {

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
			printf("%s ", (pixel[0] == 255 ? "." : (pixel[1] == 255 ? "S" : (pixel[2] == 255 ? "X" : " "))));
		}
		printf("\n");
	}
	return Graph<Point>(vertices, edges);
}

static inline bool isWall(const cv::Mat& image, int row, int col) {
	if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) {
		return true;
	}
	cv::Vec3b pixel = image.at<cv::Vec3b>(row, col);
	if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) {
		return true;
	}
	return false;
}



std::tuple<float, std::vector<Edge<Point>>> MazeSolver::solve(const Vertex<Point>& start, 
		const Vertex<Point>& end) const {
	return m_algo->solve(this->m_graph, start, end);
}

const Graph<Point>* MazeSolver::getGraph() const {
	return &m_graph;
}

Graph<Point>* MazeSolver::getGraph() {
	return &m_graph;
}