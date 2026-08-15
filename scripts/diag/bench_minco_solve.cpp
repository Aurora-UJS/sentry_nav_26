// Benchmark: colPivHouseholderQr vs closed-form for the MINCO 3x3 Hermite-quintic
// solve (solveBackpropCoeffs / solveQuintic). Compile with just Eigen:
//   g++ -O2 -I/usr/include/eigen3 bench_minco_solve.cpp -o /tmp/bench_minco_solve
#include <Eigen/Eigen>
#include <cstdio>
#include <chrono>
#include <random>

static Eigen::Vector3d backpropClosed(double T) {
    return Eigen::Vector3d(10.0 / (T * T * T),
                           -15.0 / (T * T * T * T),
                           6.0 / (T * T * T * T * T));
}
static Eigen::Vector3d backpropQR(double T) {
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    Eigen::Matrix3d A;
    A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;
    return A.colPivHouseholderQr().solve(Eigen::Vector3d(1, 0, 0));
}

int main() {
    const int N = 1000000;
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> d(0.05, 3.0);
    std::vector<double> Ts(N);
    for (auto &t : Ts) t = d(rng);

    // warm-up
    volatile double sink = 0;
    for (int i = 0; i < 1000; ++i) sink += backpropClosed(Ts[i]).norm();

    auto t0 = std::chrono::steady_clock::now();
    double s = 0;
    for (int i = 0; i < N; ++i) s += backpropClosed(Ts[i]).norm();
    auto t1 = std::chrono::steady_clock::now();
    double s2 = 0;
    for (int i = 0; i < N; ++i) s2 += backpropQR(Ts[i]).norm();
    auto t2 = std::chrono::steady_clock::now();

    double closed_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
    double qr_us = std::chrono::duration<double, std::micro>(t2 - t1).count() / N;
    std::printf("closed-form: %.3f us/call\n", closed_us);
    std::printf("QR         : %.3f us/call\n", qr_us);
    std::printf("speedup    : %.1fx\n", qr_us / closed_us);
    std::printf("(sink %g %g)\n", s, s2);
    (void)sink;
    return 0;
}
