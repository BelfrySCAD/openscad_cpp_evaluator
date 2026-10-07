// TEMPORARY: Windows-only scale(2) failure diagnosis. Not for merging.
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "test_helpers.hpp"
#include <gtest/gtest.h>
#include <sstream>
using namespace oscadeval;
using namespace oscadeval::test;
TEST(WinDebug, Scale) {
    std::ostringstream o;
    for (const char* src : {"cube(1);", "scale(2) cube(1);", "scale([2,2,2]) cube(1);", "scale(v=2) cube(1);",
                            "square(1);", "scale(2) square(1);", "scale([2,2]) square(1);",
                            "multmatrix([[2,0,0,0],[0,2,0,0],[0,0,2,0]]) cube(1);", "x=2; scale(x) cube(1);"}) {
        Evaluated e = evalSrc(src);
        o << src << "\n";
        if (!e.tree.empty()) {
            auto it = e.tree[0]->params.find("m");
            if (it != e.tree[0]->params.end()) o << "  m=" << fmtValue(it->second) << "\n";
            auto a = e.tree[0]->params.find("args");
            if (a != e.tree[0]->params.end()) o << "  args=" << fmtValue(a->second) << "\n";
        }
        for (const ColoredBody& b : e.bodies) {
            if (b.section) {
                const auto r = b.section->Bounds();
                o << "  section area=" << b.section->Area() << " x " << r.min.x << ".." << r.max.x << " y " << r.min.y << ".." << r.max.y;
                o << " xf=";
                for (int c = 0; c < 4; ++c) o << "[" << b.sectionXform[c].x << "," << b.sectionXform[c].y << "," << b.sectionXform[c].z << "]";
                o << "\n";
            } else if (b.body) {
                const auto bb = b.body->BoundingBox();
                o << "  body " << bb.min.x << ".." << bb.max.x << " " << bb.min.y << ".." << bb.max.y << " " << bb.min.z << ".." << bb.max.z << "\n";
            }
        }
    }
    ADD_FAILURE() << o.str();
}
