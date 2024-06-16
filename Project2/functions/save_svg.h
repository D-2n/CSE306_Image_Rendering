#pragma once

#include "../classes/polygon.h"
#include <fstream>
#include <iostream>
#include <vector>
void save_svg(const std::vector<Polygon> &polygons, std::string filename, std::string fillcol = "none");

void save_svg_animated(const std::vector<Polygon> &polygons, std::string filename, int frameid, int nbframes);
