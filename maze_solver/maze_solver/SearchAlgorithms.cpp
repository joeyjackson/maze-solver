#include "SearchAlgorithms.h"

std::ostream& operator<<(std::ostream &strm, const Point& pt) {
	std::stringstream s;
	s << "(" << pt.x << ", " << pt.y << ")";
	return strm << s.str();
}
