#include "math.h"

vector2 cmultiply(const vector2 a, b) {
	return set(a.x * b.x - a.y * b.y , a.x * b.y + a.y * b.x);
}

void cmultiply(const vector2 z,w; export vector2 a) {
	float x = z.x * w.x - z.y * w.y;
	a.y = z.x * w.y + z.y * w.x;
	a.x = x;
}

vector2 cinvert(const vector2 a) {
	float fac = 1/(a.x * a.x + a.y * a.y);
	return set(fac * a.x , -fac * a.y);
}

void cinvert(const vector2 a; export vector2 b) {
	float fac = 1/(a.x * a.x + a.y * a.y);
	b.x = fac * a.x;
	b.y = -fac * a.y;
}

vector2 cpow(const vector2 z; const int n) {
	vector2 w = {1,0};
	for (int j=0; j<abs(n); j++) cmultiply(w,z,w);
	if (n<0) {
		cinvert(w,w);
	}
	return w;
}

void cpow(const vector2 z; const int n; export vector2 w) {
	w.x = 1;
	w.y = 0;
	for (int j=0; j<abs(n); j++) cmultiply(w,z,w);
	if (n<0) cinvert(w,w);
}

float abs(const vector2 z) {
	return sqrt(z.x*z.x + z.y*z.y);
}