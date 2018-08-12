#include <opencv2/opencv.hpp>
#include "Graph.h"
#include <unordered_set>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
	Vertex<std::string> A(new std::string("A"));
	Vertex<std::string> B(new std::string("B"));
	Vertex<std::string> C(new std::string("C"));
	Vertex<std::string> D(new std::string("D"));

	std::unordered_set<Vertex<std::string>> verts;
	verts.insert(A);
	verts.insert(B);
	verts.insert(C);
	verts.insert(D);

	std::unordered_set<Edge<std::string>> edg;
	edg.insert(Edge<std::string>(&A, &B, 1));
	edg.insert(Edge<std::string>(&B, &C, 1));
	edg.insert(Edge<std::string>(&C, &D, 1));
	edg.insert(Edge<std::string>(&D, &A, 1));

	Graph<std::string> graph(verts, edg, true);

	std::unordered_set<Edge<std::string>>* edges = graph.getEdges();
	std::unordered_map<Vertex<std::string>, std::set<Edge<std::string>>*>* adjList = graph.getAdjList();

	std::cout << "Edge Set" << std::endl;
	for (auto it = edges->begin(); it != edges->end(); ++it) {
		std::cout << *(it->getU()->getData()) << " " << *(it->getV()->getData()) << " " << it->getWeight() << std::endl;
	}

	std::cout << "Adjacency List" << std::endl;
	for (auto it = adjList->begin(); it != adjList->end(); ++it) {
		std::cout << *(it->first.getData()) << "(" << it->second->size() << ")" << " : ";
		for (auto e_it = it->second->begin(); e_it != it->second->end(); ++e_it) {
			std::cout << *(e_it->getV()->getData()) << " ";
		}
		std::cout << std::endl;
	}

	std::cin.get();
	return 0;
}