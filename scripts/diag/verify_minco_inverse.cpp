// Standalone verification for the closed-form 3x3 Hermite-quintic inverse used in
// MincoTrajectory::solveBackpropCoeffs / solveQuintic. Compile with just Eigen:
//   g++ -O2 -I/usr/include/eigen3 verify_minco_inverse.cpp -o /tmp/verify_minco_inverse
//
// Correctness is judged by RESIDUALS (A*x = b), not by comparing against QR:
// for small T the matrix is ill-conditioned (det = 2T^9 -> 0), so QR/LU roundoff
// is the "wrong" reference — the closed form is the analytic truth.
#include <Eigen/Eigen>
#include <cstdio>
#include <cmath>
#include <random>

static Eigen::Vector3d backpropClosed(double T) {
    Eigen::Vector3d bc;
    bc(0) = 10.0 / (T * T * T);
    bc(1) = -15.0 / (T * T * T * T);
    bc(2) = 6.0 / (T * T * T * T * T);
    return bc;
}

static Eigen::Vector3d solveClosed(double T, const Eigen::Vector3d &b) {
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    Eigen::Vector3d x;
    x(0) = (10.0 / T3) * b(0) - (4.0 / T2) * b(1) + (0.5 / T) * b(2);
    x(1) = -(15.0 / T4) * b(0) + (7.0 / T3) * b(1) - (1.0 / T2) * b(2);
    x(2) = (6.0 / T5) * b(0) - (3.0 / T4) * b(1) + (0.5 / T3) * b(2);
    return x;
}

static Eigen::Matrix3d buildA(double T) {
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    Eigen::Matrix3d A;
    A << T3, T4, T5,
         3 * T2, 4 * T3, 5 * T4,
         6 * T, 12 * T2, 20 * T3;
    return A;
}

int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> Tdist(0.05, 3.0);
    std::uniform_real_distribution<double> bdist(-5.0, 5.0);

    double max_res_inv = 0.0, max_res_bp = 0.0, max_res_solve = 0.0;
    for (int it = 0; it < 200000; ++it) {
        double T = Tdist(rng);
        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
        Eigen::Matrix3d A = buildA(T);

        // 1) A * Ainv_closed == I
        Eigen::Matrix3d Ainv;
        Ainv << 10.0 / T3, -4.0 / T2, 0.5 / T,
               -15.0 / T4, 7.0 / T3, -1.0 / T2,
                6.0 / T5, -3.0 / T4, 0.5 / T3;
        double r = (A * Ainv - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
        if (r > max_res_inv) max_res_inv = r;

        // 2) A * backpropClosed == [1,0,0]
        Eigen::Vector3d e1(1.0, 0.0, 0.0);
        Eigen::Vector3d bp = backpropClosed(T);
        r = (A * bp - e1).cwiseAbs().maxCoeff();
        if (r > max_res_bp) max_res_bp = r;

        // 3) A * solveClosed == b
        Eigen::Vector3d b(bdist(rng), bdist(rng), bdist(rng));
        Eigen::Vector3d x = solveClosed(T, b);
        r = (A * x - b).cwiseAbs().maxCoeff();
        if (r > max_res_solve) max_res_solve = r;
    }

    std::printf("max ||A*Ainv - I||_inf  = %.3e\n", max_res_inv);
    std::printf("max ||A*backprop - e1||  = %.3e\n", max_res_bp);
    std::printf("max ||A*solveQuintic - b||= %.3e\n", max_res_solve);
    double tol = 1e-8;  // residual should be ~machine-eps relative
    bool ok = max_res_inv < tol && max_res_bp < tol && max_res_solve < tol;
    std::printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
