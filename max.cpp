/*
 * Copyright 2023 Ethan. All rights reserved.
 */
#include "max.h"
#include <cmath>
#include <limits>

constexpr double EPSILON = std::numeric_limits<double>::epsilon() * 1000000;

static bool approx_equal(double a, double b, double epsilon = EPSILON) {
	const double abs_a = std::fabs(a);
	const double abs_b = std::fabs(b);
	const double diff = std::fabs(a - b);

	if (a == b) {
		return true;
	} else if (a == 0 || b == 0 || (abs_a + abs_b < std::numeric_limits<double>::min())) {
		return diff < (epsilon * std::numeric_limits<double>::min());
	} else {
		return diff / (abs_a + abs_b) < epsilon;
	}
}

double max(double a, double b) {
	if (std::isnan(a)) return b;
	if (std::isnan(b)) return a;

	if (approx_equal(a, b)) {
		return a;
	} else if (a > b) {
		return a;
	} else {
		return b;
	}
}
