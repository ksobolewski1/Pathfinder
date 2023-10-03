#pragma once

#include "NavMesh.h"


struct A_Star {

private:

	// Wraps data specific to the A* algorithm 
	struct Node {

		const NavMesh::NodeData data;
		Node* parent = nullptr; 

		Node() = default; 
		Node(const NavMesh::NodeData& d) : data(d) {}

		float gcost_scalar = 0.42f; 
		float h_cost = 0.0f; // distance from this node to destination node
		float g_cost = 0.0f; // the total cost of the path taken from the start to reach this node 

		void SetHCost(const sf::Vector2f& to_dest) { h_cost = (float)std::sqrt(to_dest.x * to_dest.x + to_dest.y * to_dest.y); }
	};

public:

	static std::vector<int> Find(const NavMesh& mesh);

};


// Binary heap for queueing Nodes in A* 
template <typename T, typename ID>
struct Heap
{
private:
	int N = 0;
	std::vector<T*> vect = { nullptr };
	bool(*comparator)(T*, T*);

	int GetParent(int index) { return index >> 1; }
	int GetLeftChild(int index) { return index << 1; }
	int GetRightChild(int index) { return (index << 1) + 1; }

public:

	Heap() = default;
	Heap(bool(*comp)(T*, T*), size_t max = 0) : comparator(comp) { vect.reserve(max); }

	bool Empty() { return N == 0; }
	int GetSize() { return N; }

	T* GetRoot() {
		if (!Empty()) return vect[1];
		else return nullptr;
	}

	void Insert(T* el) {
		vect.emplace_back(el);
		++N;
		HeapUp(N);
	}

	T* RemoveRoot() {

		if (Empty()) return nullptr;

		std::swap(vect[1], vect[N]);
		T* val = vect[N];
		vect.pop_back();
		--N;
		HeapDown(1);

		return val;
	}

	void HeapUp(int i) {
		if (i > N || i == 1) return;

		int index = GetParent(i);
		if (comparator(vect[i], vect[index])) std::swap(vect[index], vect[i]);
		HeapUp(index);
	}

	void HeapDown(int i) {
		if (i > N) return;

		int prev = i;
		if (GetLeftChild(i) <= N && comparator(vect[i], vect[GetLeftChild(i)])) prev = GetLeftChild(i);
		if (GetRightChild(i) <= N && comparator(vect[GetRightChild(i)], vect[prev])) prev = GetRightChild(i);

		if (prev != i) {
			std::swap(vect[i], vect[prev]);
			HeapDown(prev);
		}
	}
};




