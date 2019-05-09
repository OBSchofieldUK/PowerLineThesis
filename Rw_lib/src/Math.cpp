/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "rw/math/Math.hpp"
//#include "MetricFactory.hpp"
#include "rw/math/Random.hpp"
#include <rw/common/macros.hpp>

#include <boost/math/special_functions/fpclassify.hpp> // boost::math::isnan()

#include <cmath>
#include <limits>

using namespace rw::math;


double Math::ranNormalDist(double mean, double sigma) {
	return Random::ranNormalDist(mean,sigma);
}

double Math::ran() {
	return Random::ran();
}

void Math::seed(unsigned seed) {
    Random::seed(seed);
}

void Math::seed() {
	Random::seed();
}

double Math::ran(double from, double to) {
    return Random::ran(from,to);
}

int Math::ranI(int from, int to) {
	return Random::ranI(from,to);
}

int Math::ceilLog2(const int n)
{
    RW_ASSERT(n > 0);
    int cnt = 0;
    int i = n;
    int a = 1;
    while (i != 1) {
        a <<= 1;
        i >>= 1;
        ++cnt;
    }
    if (a == n)
        return cnt;
    else
        return cnt + 1;
}

bool Math::isNaN(double d) {
	return boost::math::isnan(d);
}

double Math::NaN() {
    return std::numeric_limits<double>::quiet_NaN();
}
