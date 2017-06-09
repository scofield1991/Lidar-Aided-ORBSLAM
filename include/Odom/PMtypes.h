#ifndef PMTYPES_H
#define PMTYPES_H

#define protected public
#include "pointmatcher/PointMatcher.h"
#undef protected
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;

typedef typename Nabo::NearestNeighbourSearch<float> NNS;
typedef typename NNS::SearchType NNSearchType;

#endif