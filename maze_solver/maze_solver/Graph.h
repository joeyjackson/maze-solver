#pragma once

#include <unordered_set>
#include <unordered_map>
#include <set>
#include <iostream>
#include <memory>

template <typename T>
class Vertex {
private:
	std::shared_ptr<T> data;
public:
	Vertex();
	Vertex(std::shared_ptr<T> data_);
	std::shared_ptr<T> getData() const;
	bool operator==(const Vertex<T>& other) const;
	bool operator>(const Vertex<T>& other) const;
	bool operator<(const Vertex<T>& other) const;
};

template<typename T> 
struct std::hash<Vertex<T>> {
	size_t operator()(const Vertex<T>& obj) const;
};

template <typename T>
class Edge {
private:
	std::shared_ptr<Vertex<T>> u, v;
	float weight;
public:
	Edge(std::shared_ptr<Vertex<T>> u_, std::shared_ptr<Vertex<T>> v_, float weight_);
	std::shared_ptr<Vertex<T>> getU() const;
	void setU(std::shared_ptr<Vertex<T>> new_u);
	std::shared_ptr<Vertex<T>> getV() const;
	void setV(std::shared_ptr<Vertex<T>> new_v);
	float getWeight() const;
	void setWeight(float new_weight);
	bool operator<(const Edge<T>& other) const;
	bool operator>(const Edge<T>& other) const;
	bool operator==(const Edge<T>& other) const;
};

template<typename T>
struct std::hash<Edge<T>> {
	size_t operator()(const Edge<T>& obj) const;
};

template <typename T>
class Graph {
private:
	std::unordered_set<Edge<T>> edges;
	std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*> adjList;
	bool undirected;
public:
	explicit Graph(bool undirected_ = false);
	explicit Graph(const std::unordered_set<Vertex<T>>& vertices_, const std::unordered_set<Edge<T>>& edges_, bool undirected_ = false);
	~Graph();
	Graph(const Graph& other);
	Graph& operator=(const Graph& other);
	std::unordered_set<Edge<T>>* getEdges();
	std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*>* getAdjList();
	const std::unordered_set<Edge<T>>* getEdges() const;
	const std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*>* getAdjList() const;
	void addEdge(const Edge<T>& edge);
	void addVertex(const Vertex<T>& vertex);
};



//***************** VERTEX ****************************************************

template <typename T>
Vertex<T>::Vertex() {}

template <typename T>
Vertex<T>::Vertex(std::shared_ptr<T> data_) : data(data_) {}

template <typename T>
std::shared_ptr<T> Vertex<T>::getData() const { return data; }

template <typename T>
bool Vertex<T>::operator==(const Vertex<T>& other) const { return *(this->data) == *(other.data); }

template <typename T>
bool Vertex<T>::operator>(const Vertex<T>& other) const { return *(this->data) > *(other.data); }

template <typename T>
bool Vertex<T>::operator<(const Vertex<T>& other) const { return *(this->data) < *(other.data); }

template<typename T>
size_t std::hash<Vertex<T>>::operator()(const Vertex<T>& obj) const {
	return hash<T>()(*(obj.getData()));
}

//***************** EDGE ******************************************************
template <typename T>
Edge<T>::Edge(std::shared_ptr<Vertex<T>> u_, std::shared_ptr<Vertex<T>> v_, float weight_) : u(u_), v(v_), weight(weight_) { };

template <typename T>
std::shared_ptr<Vertex<T>> Edge<T>::getU() const { return u; }

template <typename T>
void Edge<T>::setU(std::shared_ptr<Vertex<T>> new_u) { u = new_u; }

template <typename T>
std::shared_ptr<Vertex<T>> Edge<T>::getV() const { return v; }

template <typename T>
void Edge<T>::setV(std::shared_ptr<Vertex<T>> new_v) { v = new_v; }

template <typename T>
float Edge<T>::getWeight() const { return weight; }

template <typename T>
void Edge<T>::setWeight(float new_weight) { weight = new_weight; }

template <typename T>
bool Edge<T>::operator<(const Edge<T>& other) const { 
	if (this->weight < other.getWeight()) {
		return true;
	}
	return false;
}

template <typename T>
bool Edge<T>::operator>(const Edge<T>& other) const { 
	return this->weight > other.getWeight(); 
}

template <typename T>
bool Edge<T>::operator==(const Edge<T>& other) const {
	return *(this->u) == *other.getU() && *(this->v) == *other.getV() && this->weight == other.getWeight();
}

template<typename T>
size_t std::hash<Edge<T>>::operator()(const Edge<T>& obj) const {
	std::hash<Vertex<T>> v_hash;
	return (34 * (34 * v_hash(*(obj.getU())) + v_hash(*(obj.getU())))) + obj.getWeight();
}

//***************** GRAPH *****************************************************
template <typename T>
Graph<T>::Graph(bool undirected_) : undirected(undirected_) {
}

template <typename T>
Graph<T>::Graph(const std::unordered_set<Vertex<T>>& vertices_, const std::unordered_set<Edge<T>>& edges_, bool undirected_) : edges(edges_), undirected(undirected_) {
	if (undirected) {
		for (auto it = edges_.begin(); it != edges_.end(); ++it) {
			edges.insert(Edge<T>(it->getV(), it->getU(), it->getWeight()));
		}
	}
	for (std::unordered_set<Vertex<T>>::iterator it = vertices_.begin(); it != vertices_.end(); ++it) {
		adjList.emplace(*it, new std::unordered_set<Edge<T>>);
	}
	for (auto it = edges.begin(); it != edges.end(); ++it) {
		if (adjList.count(*(it->getU()))) {
			adjList.at(*(it->getU()))->insert(*it);
		}
	}
}

template <typename T>
Graph<T>::~Graph() {
	for (auto it = adjList.begin(); it != adjList.end(); ++it) {
		delete it->second;
	}
}

template <typename T>
Graph<T>::Graph(const Graph& other) : edges(other.edges) {
	for (auto it = other.getAdjList()->begin(); it != other.getAdjList()->end(); ++it) {
		this->adjList.emplace(it->first, new std::unordered_set<Edge<T>>(*(it->second)));
	}
}

template <typename T>
Graph<T>& Graph<T>::operator=(const Graph& other) {

	edges = *(other.getEdges());
	for (auto it = other.getAdjList()->begin(); it != other.getAdjList()->end(); ++it) {
		this->adjList.emplace(it->first, new std::unordered_set<Edge<T>>(*(it->second)));
	}
	return *this;
}

template <typename T>
std::unordered_set<Edge<T>>* Graph<T>::getEdges() {
	return &(edges);
}

template <typename T>
std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*>* Graph<T>::getAdjList() {
	return &(adjList);
}

template <typename T>
const std::unordered_set<Edge<T>>* Graph<T>::getEdges() const {
	return &(edges);
}

template <typename T>
const std::unordered_map<Vertex<T>, std::unordered_set<Edge<T>>*>* Graph<T>::getAdjList() const {
	return &(adjList);
}

template <typename T>
void Graph<T>::addEdge(const Edge<T>& edge) {
	edges.insert(edge);
	if (undirected) {
		edges.insert( Edge<T>(edge.getV(), edge.getU(), edge.getWeight()) );
	}

	if (!adjList.count(*(edge.getU()))) {
		adjList.emplace(*(edge.getU()), new std::unordered_set<Edge<T>>);
	}
	if (!adjList.count(*(edge.getV()))) {
		adjList.emplace(*(edge.getV()), new std::unordered_set<Edge<T>>);
	}
	adjList.at(*(edge.getU()))->insert(edge);
	if (undirected) {
		adjList.at( *(edge.getV()) )->insert( Edge<T>(edge.getV(), edge.getU(), edge.getWeight()) );
	}
}

template <typename T>
void Graph<T>::addVertex(const Vertex<T>& vertex) {
	if (!adjList.count(vertex)) {
		adjList.emplace(vertex, new std::unordered_set<Edge<T>>);
	}
}

