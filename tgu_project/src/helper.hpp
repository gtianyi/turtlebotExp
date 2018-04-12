#pragma once
/**
 * File Name:   helper.hpp
 * Author:      Tianyi Gu, Zhuo Xu
 * Assignment:  Final Project - Frontier Based Exploration
 * Date:        April-8-2018 
 * Descroption: Helpe functions 
 */

#include <cmath>
#include <vector>

// Bresenham's line algorithm
std::vector<std::pair<int, int>> getLine(const int x1i,
        const int y1i,
        const int x2i,
        const int y2i) {
    std::vector<std::pair<int, int>> ret;

    float x1 = (float)x1i;
    float y1 = (float)y1i;
    float x2 = (float)x2i;
    float y2 = (float)y2i;

    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for (int x = (int)x1; x < maxX; x++) {
        if (steep) {
            ret.push_back(std::pair<int, int>(y, x));
        } else {
            ret.push_back(std::pair<int, int>(x, y));
        }

        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }

	return ret;
};
