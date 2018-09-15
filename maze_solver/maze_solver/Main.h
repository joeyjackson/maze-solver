#pragma once

#include "Graph.h"
#include "SearchAlgorithms.h"
#include "MazeSolver.h"
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

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

void image_resize(const cv::Mat& img, cv::Mat& dst, int height, int width, int inter = cv::INTER_LINEAR);