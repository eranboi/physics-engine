#pragma once
#include <vector>
#include "../../Core/Rigidbody.h"

class Grid {
public:
	Grid(float width, float height, float cellSize);
	void Clear();
	void AddBody(Rigidbody* body);
	const std::vector<Rigidbody*>& GetCellContent(int x, int y) const;

	float GetCellSize() const { return cellSize; }

private:
	float width;
	float height;
	float cellSize;
	int cols;
	int rows;

	std::vector<std::vector<Rigidbody*>> cells;

};