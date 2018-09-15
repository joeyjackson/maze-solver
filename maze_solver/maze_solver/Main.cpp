#include "Graph.h"
#include "SearchAlgorithms.h"
#include "MazeSolver.h"
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

template <typename T>
void PrintGraph(const Graph<T>& graph) {
	const std::unordered_set<Edge<T>>* edges = graph.getEdges();
	const std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*>* adjList = graph.getAdjList();
	std::cout << "Edge Set" << std::endl;
	for (auto it = edges->begin(); it != edges->end(); ++it) {
		std::cout << (*(it->getU()->getData())) << " " << (*(it->getV()->getData())) << " " << it->getWeight() << std::endl;
	}

	std::cout << "Adjacency List" << std::endl;
	for (auto it = adjList->begin(); it != adjList->end(); ++it) {
		std::cout << (*(it->first.getData())) << "(" << it->second->size() << ")" << " : ";
		for (auto e_it = it->second->begin(); e_it != it->second->end(); ++e_it) {
			std::cout << (*(e_it->getV()->getData())) << " ";
		}
		std::cout << std::endl;
	}
}

int main(int argc, char** argv) {
	/*
	Vertex<Point> A(std::make_shared<Point>(1,1));
	Vertex<Point> B(std::make_shared<Point>(1,2));
	Vertex<Point> C(std::make_shared<Point>(1,3));
	Vertex<Point> D(std::make_shared<Point>(1,4));
	Vertex<Point> E(std::make_shared<Point>(1,5));
	Vertex<Point> F(std::make_shared<Point>(1,6));
	Vertex<Point> G(std::make_shared<Point>(1,7));
	Vertex<Point> H(std::make_shared<Point>(1,8));
	Vertex<Point> I(std::make_shared<Point>(1,9));

	std::unordered_set<Vertex<Point>> verts;
	verts.insert(A);
	verts.insert(B);
	verts.insert(C);
	verts.insert(D);
	verts.insert(E);
	verts.insert(F);
	verts.insert(G);
	verts.insert(H);
	verts.insert(I);

	std::unordered_set<Edge<Point>> edg;
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(A), std::make_shared<Vertex<Point>>(B), 4));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(A), std::make_shared<Vertex<Point>>(C), 9));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(A), std::make_shared<Vertex<Point>>(D), 8));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(B), std::make_shared<Vertex<Point>>(D), 5));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(B), std::make_shared<Vertex<Point>>(E), 7));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(C), std::make_shared<Vertex<Point>>(D), 2));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(C), std::make_shared<Vertex<Point>>(F), 1));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(D), std::make_shared<Vertex<Point>>(G), 10));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(E), std::make_shared<Vertex<Point>>(G), 6));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(F), std::make_shared<Vertex<Point>>(G), 3));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(F), std::make_shared<Vertex<Point>>(I), 19));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(G), std::make_shared<Vertex<Point>>(I), 8));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(G), std::make_shared<Vertex<Point>>(H), 12));
	edg.insert(Edge<Point>(std::make_shared<Vertex<Point>>(I), std::make_shared<Vertex<Point>>(H), 13));*/

	//Graph<Point> graph(verts, edg, true);
	//PrintGraph(graph);

	/*auto solution = solver.solve(A, H);
	float length = std::get<0>(solution);
	std::cout << length << std::endl;
	if (length >= 0) {

		auto path = std::get<1>(solution);
		for (auto it = path.begin(); it != path.end(); ++it) {
			std::cout << (*it->getU()->getData()) << " ";
		}
		std::cout << (*path.back().getV()->getData()) << std::endl;
	}*/

	cv::Mat image = cv::imread("C:\\Users\\ejjac\\Documents\\CppProjs\\OpenCV_Proj\\maze-solver\\maze1.png");

	MazeSolver solver(image);

	//PrintGraph(*(solver.getGraph()));

	cv::Mat newImg = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
	printf("%d", solver.getGraph()->getEdges()->size());
	for (Edge<Point> edge : *(solver.getGraph()->getEdges())) {
		std::cout << *(edge.getU()->getData()) << std::endl;

		cv::Point p_u(edge.getU()->getData()->x, edge.getU()->getData()->y);
		cv::Point p_v(edge.getV()->getData()->x, edge.getV()->getData()->y);
		//cv::line(newImg, p_u, p_v, cv::Scalar(255, 255, 255), 1);
		cv::Point3_<uchar>* p1 = newImg.ptr<cv::Point3_<uchar> >(p_u.y, p_u.x);
		p1->x = 255;
		cv::Point3_<uchar>* p2 = newImg.ptr<cv::Point3_<uchar> >(p_v.y, p_v.x);
		p2->x = 255;
	}



	cv::resize(newImg, newImg, cv::Size(200, 200));

	cv::imshow("test", newImg);

	std::cin.get();
	return 0;
}