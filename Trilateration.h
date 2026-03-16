#ifndef TRILATERATION_H
#define TRILATERATION_H

#include <Arduino.h>
#include <map>
#include <vector>
#include <math.h>

struct Point {
    float x;
    float y;
};

struct Anchor {
    uint16_t id; // Short Address
    float x;
    float y;
    float distance;
    bool hasDistance;
};

class Trilateration {
private:
    std::map<uint16_t, Anchor> anchors;

public:
    Trilateration() {}

    // Add or update an anchor's known position
    void addAnchor(uint16_t id, float x, float y) {
        anchors[id] = {id, x, y, 0.0, false};
    }

    // Update the distance to a specific anchor
    void updateDistance(uint16_t id, float dist) {
        if (anchors.find(id) != anchors.end()) {
            anchors[id].distance = dist;
            anchors[id].hasDistance = true;
        }
    }

    // Check if we have valid ranges for at least 3 anchors
    bool isReady() {
        int count = 0;
        for (auto const& [id, anchor] : anchors) {
            if (anchor.hasDistance) count++;
        }
        return count >= 3;
    }

    // Calculate position using Linear Least Squares
    // Returns true if successful, false if singular matrix or not enough data
    bool calculatePosition(Point& result) {
        std::vector<Anchor> validAnchors;
        for (auto const& [id, anchor] : anchors) {
            if (anchor.hasDistance) {
                validAnchors.push_back(anchor);
            }
        }

        if (validAnchors.size() < 3) return false;

        // Use the last anchor as the reference to linearize
        Anchor ref = validAnchors.back();
        validAnchors.pop_back();

        // System matrix A (Size: N x 2) and vector b (Size: N)
        // A * [x, y]^T = b
        // We accumulate A^T * A and A^T * b to solve (A^T A) * p = A^T b directly (Normal Equations)
        
        float AtA_00 = 0;
        float AtA_01 = 0; // Symmetric AtA_10
        float AtA_11 = 0;
        
        float Atb_0 = 0;
        float Atb_1 = 0;

        for (const auto& ank : validAnchors) {
            // Equation: 2x(xn - xi) + 2y(yn - yi) = (di^2 - ri^2) - (dn^2 - rn^2)
            // A_row = [ 2(xn - xi),  2(yn - yi) ]
            // b_val = (di^2 - xi^2 - yi^2) - (dn^2 - xn^2 - yn^2)

            float A = 2 * (ref.x - ank.x);
            float B = 2 * (ref.y - ank.y);
            
            float r_i_sq = ank.x * ank.x + ank.y * ank.y;
            float r_n_sq = ref.x * ref.x + ref.y * ref.y;
            
            float C = (ank.distance * ank.distance - r_i_sq) - (ref.distance * ref.distance - r_n_sq);

            // Accumulate for Normal Equations: (A^T A) and (A^T b)
            AtA_00 += A * A;
            AtA_01 += A * B;
            AtA_11 += B * B;
            
            Atb_0 += A * C;
            Atb_1 += B * C;
        }

        // Solve (A^T A) * [x, y] = (A^T b) for [x, y]
        // Determinant
        float det = AtA_00 * AtA_11 - AtA_01 * AtA_01;

        if (fabs(det) < 1e-6) return false; // Singular matrix (collinear anchors?)

        // Inverse of 2x2:
        // [ x ] = (1/det) * [  AtA_11  -AtA_01 ] * [ Atb_0 ]
        // [ y ]             [ -AtA_01   AtA_00 ]   [ Atb_1 ]

        result.x = (AtA_11 * Atb_0 - AtA_01 * Atb_1) / det;
        result.y = (-AtA_01 * Atb_0 + AtA_00 * Atb_1) / det;

        return true;
    }
};

#endif
