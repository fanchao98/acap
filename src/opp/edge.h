#pragma once
#ifndef EDGE_H_
#define EDGE_H_


class Edge 
{
private:
	int from, to;
public:
	Edge() {};
	Edge(int _from, int _to) : from(_from), to(_to) {};
	inline int GetFrom() { return from; }
	inline int GetTo() { return to; }
};

#endif