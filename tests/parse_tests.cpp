#include <cassert>
#include <cmath>
#include "utils.h"

int main() {
    int w = 0, h = 0;
    bool ok = autonomous_driving::extractWidthHeight<int>("1920x1080", w, h);
    assert(ok);
    assert(w == 1920 && h == 1080);

    float wf = 0.f, hf = 0.f;
    ok = autonomous_driving::extractWidthHeight<float>("10.5x20.25", wf, hf);
    assert(ok);
    assert(std::fabs(wf - 10.5f) < 1e-6f);
    assert(std::fabs(hf - 20.25f) < 1e-6f);

    int badW = 0, badH = 0;
    ok = autonomous_driving::extractWidthHeight<int>("1920-1080", badW, badH);
    assert(!ok);

    return 0;
}
