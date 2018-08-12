#pragma once
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <iostream>

template <typename T>
class Vertex {
private:
	T* data;
public:
	Vertex(T* data_);
	T* getData() const;
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
	Vertex<T> *u, *v;
	unsigned int weight;
public:
	Edge(Vertex<T>* u_, Vertex<T>* v_, unsigned int weight_);
	Vertex<T>* getU() const;
	Vertex<T>* getV() const;
	unsigned int getWeight() const;
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
	std::unordered_map<Vertex<T>, std::set<Edge<T>>*> adjList;
	bool undirected;
public:
	explicit Graph(bool undirected_ = false);
	explicit Graph(const std::unordered_set<Vertex<T>>& vertices_, const std::unordered_set<Edge<T>>& edges_, bool undirected_ = false);
	std::unordered_set<Edge<T>>* getEdges();
	std::unordered_map<Vertex<T>, std::set<Edge<T>>*>* getAdjList();
};



//***************** VERTEX ****************************************************
template <typename T>
Vertex<T>::Vertex(T* data_) : data(data_) {}

template <typename T>
T* Vertex<T>::getData() const { return data; }

template <typename T>
bool Vertex<T>::operator==(const Vertex<T>& other) const { return *(this->data) == (*other.getData()); }

template <typename T>
bool Vertex<T>::operator>(const Vertex<T>& other) const { return *(this->data) > (*other.getData()); }

template <typename T>
bool Vertex<T>::operator<(const Vertex<T>& other) const { return *(this->data) < (*other.getData()); }

template<typename T>
size_t std::hash<Vertex<T>>::operator()(const Vertex<T>& obj) const {
	return hash<T>()(*(obj.getData()));
}

//***************** EDGE ******************************************************
template <typename T>
Edge<T>::Edge(Vertex<T>* u_, Vertex<T>* v_, unsigned int weight_) : u(u_), v(v_), weight(weight_) {};

template <typename T>
Vertex<T>* Edge<T>::getU() const { return u; }

template <typename T>
Vertex<T>* Edge<T>::getV() const { return v; }

template <typename T>
unsigned int Edge<T>::getWeight() const { return weight; }

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
Graph<T>::Graph(bool undirected_) : undirected(undirected_) {}

template <typename T>
Graph<T>::Graph(const std::unordered_set<Vertex<T>>& vertices_, const std::unordered_set<Edge<T>>& edges_, bool undirected_) : edges(edges_), undirected(undirected_) {
	if (undirected) {
		for (auto it = edges_.begin(); it != edges_.end(); ++it) {
			edges.insert(Edge<T>(it->getV(), it->getU(), it->getWeight()));
		}
	}
	for (std::unordered_set<Vertex<T>>::iterator it = vertices_.begin(); it != vertices_.end(); ++it) {
		adjList.emplace(*it, new std::set<Edge<T>>);
	}
	for (auto it = edges.begin(); it != edges.end(); ++it) {
		if (adjList.count(*(it->getU()))) {
			adjList.at(*(it->getU()))->insert(*it);
		}
	}
}

template <typename T>
std::unordered_set<Edge<T>>* Graph<T>::getEdges() {
	return &(edges);
}

template <typename T>
std::unordered_map<Vertex<T>, std::set<Edge<T>>*>* Graph<T>::getAdjList() {
	return &(adjList);
}