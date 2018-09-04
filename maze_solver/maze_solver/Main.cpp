#include <opencv2/opencv.hpp>
#include "Graph.h"
#include "SearchAlgorithms.h"
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

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
	
	Vertex<Point> A(new Point(1,1));
	Vertex<Point> B(new Point(1,2));
	Vertex<Point> C(new Point(1,3));
	Vertex<Point> D(new Point(1,4));
	Vertex<Point> E(new Point(1,5));
	Vertex<Point> F(new Point(1,6));
	Vertex<Point> G(new Point(1,7));
	Vertex<Point> H(new Point(1,8));
	Vertex<Point> I(new Point(1,9));

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
	edg.insert(Edge<Point>(&A, &B, 4));
	edg.insert(Edge<Point>(&A, &C, 9));
	edg.insert(Edge<Point>(&A, &D, 8));
	edg.insert(Edge<Point>(&B, &D, 5));
	edg.insert(Edge<Point>(&B, &E, 7));
	edg.insert(Edge<Point>(&C, &D, 2));
	edg.insert(Edge<Point>(&C, &F, 1));
	edg.insert(Edge<Point>(&D, &G, 10));
	edg.insert(Edge<Point>(&E, &G, 6));
	edg.insert(Edge<Point>(&F, &G, 3));
	edg.insert(Edge<Point>(&F, &I, 19));
	edg.insert(Edge<Point>(&G, &I, 8));
	edg.insert(Edge<Point>(&G, &H, 12));
	edg.insert(Edge<Point>(&I, &H, 13));

	Graph<Point> graph(verts, edg, true);
	//PrintGraph(graph);

	AStarSearch<Point> ds(Point::euclidean_distance);
	auto solution = ds.solve(graph, A, H);
	float length = std::get<0>(solution);
	std::cout << length << std::endl;
	if (length >= 0) {

		auto path = std::get<1>(solution);
		for (auto it = path.begin(); it != path.end(); ++it) {
			std::cout << (*it->getU()->getData()) << " ";
		}
		std::cout << (*path.back().getV()->getData()) << std::endl;
	}

	std::cin.get();
	return 0;
}