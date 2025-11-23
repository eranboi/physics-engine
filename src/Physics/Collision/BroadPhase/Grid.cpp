#include "Grid.h"
#include <cmath>

Grid::Grid(const float width, const float height, const float cellSize)
	: width(width), height(height), cellSize(cellSize) {

	// Calculate the cols and rows amount
	cols = static_cast<int>(std::ceil(width / cellSize));
	rows = static_cast<int>(std::ceil(height / cellSize));

	// allocate enough space
	cells.resize(cols * rows);
}


void Grid::Clear() {
	for (auto& cell : cells) {
		cell.clear();
	}
}

void Grid::AddBody(Rigidbody* body) {
	// Get the grid x and y from world position of the body.
	int col = static_cast<int>(body->position.x / cellSize);
	int row = static_cast<int>(body->position.y / cellSize);

	// Clamp the body pos to cols & rows
	if (col < 0) col = 0;
	else if (col >= cols) col = cols - 1;

	if (row < 0) row = 0;
	else if (row >= rows) row = rows - 1;

	// Flatten the index to 1D
	int index = col + (cols * row);

	// Push the body to the cell
	cells[index].push_back(body);
}

const std::vector<Rigidbody*>& Grid::GetCellContent(const int x, const int y) const {
	if (x < 0 || x >= cols || y < 0 || y >= rows) {
		static std::vector<Rigidbody*> empty;
		return empty;
	}
	// Flatten the index
	int index = x + (cols * y);

	return cells[index];

}
