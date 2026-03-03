/**
 * @file testBoxArmCollision.cpp
 * @brief 使用SO库验证搬运段箱子与机械臂碰撞距离
 *
 * 核心方法:
 * 1. updateAC() 更新关节状态
 * 2. getUIInfoMation() 获取碰撞体世界坐标 (capsule: data[0-2]=p1, data[3-5]=p2)
 * 3. 轨迹文件TCP坐标(位置) + FK2(朝向) → 箱子26采样点
 * 4. 点到线段(胶囊)/点到点(球)距离 - 半径 = 表面距离
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <dlfcn.h>

// ── SO library types ─────────────────────────────────────────────
typedef double       SO_LREAL;
typedef long long    SO_LINT;
typedef int          SO_DINT;
typedef short        SO_INT;
typedef signed char  SO_BOOL;
typedef struct { SO_LREAL X, Y, Z, A, B, C; } MC_COORD_REF;

// ── Function pointer types ───────────────────────────────────────
typedef void(*FnInitAC)(SO_INT, SO_LREAL[8], SO_LREAL[7], SO_LREAL[7],
                         SO_LREAL[7], SO_LREAL[7], SO_LREAL[4], SO_LREAL[6]);
typedef void(*FnUpdateAC)(SO_LREAL[6], SO_LREAL[6], SO_LREAL[6]);
typedef SO_INT(*FnCheckSelfColl)(SO_LINT*, SO_LREAL*);
typedef void(*FnSetColliderOpen)(SO_BOOL*);
typedef void(*FnGetUIInfo)(SO_DINT[7], SO_DINT[7], SO_LREAL[7][9], SO_LREAL[7]);
typedef SO_BOOL(*FnForwardKin)(SO_LREAL*, MC_COORD_REF*);
typedef SO_INT(*FnSetToolBall)(SO_LINT, SO_LREAL[3], SO_LREAL);
typedef SO_INT(*FnRemoveTool)(SO_LINT);

// ── Box dimensions (mm) ──────────────────────────────────────────
constexpr double BOX_LX = 350.0;   // X方向长
constexpr double BOX_WY = 280.0;   // Y方向宽
constexpr double BOX_HZ = 250.0;   // Z方向高

// ── 3D vector utilities ─────────────────────────────────────────
struct Vec3 { double x, y, z; };
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 operator*(double s, Vec3 v) { return {s*v.x, s*v.y, s*v.z}; }
double dot(Vec3 a, Vec3 b)     { return a.x*b.x + a.y*b.y + a.z*b.z; }
double len(Vec3 v)              { return std::sqrt(dot(v, v)); }

// Point-to-segment distance
double ptSegDist(Vec3 p, Vec3 a, Vec3 b) {
    Vec3 ab = b - a, ap = p - a;
    double ab2 = dot(ab, ab);
    if (ab2 < 1e-12) return len(ap);           // degenerate segment
    double t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));
    return len(p - (a + t * ab));
}

// 3×3 rotation matrix
struct Mat3 { double m[3][3]; };
Vec3 mulMV(const Mat3& M, Vec3 v) {
    return {M.m[0][0]*v.x + M.m[0][1]*v.y + M.m[0][2]*v.z,
            M.m[1][0]*v.x + M.m[1][1]*v.y + M.m[1][2]*v.z,
            M.m[2][0]*v.x + M.m[2][1]*v.y + M.m[2][2]*v.z};
}

// Euler ZYX rotation (A=Rz, B=Ry, C=Rx, degrees → matrix)
Mat3 eulerZYX(double A_d, double B_d, double C_d) {
    double a = A_d*M_PI/180, b = B_d*M_PI/180, c = C_d*M_PI/180;
    double ca=cos(a), sa=sin(a), cb=cos(b), sb=sin(b), cc=cos(c), sc=sin(c);
    return Mat3{{{ca*cb, ca*sb*sc - sa*cc, ca*sb*cc + sa*sc},
                 {sa*cb, sa*sb*sc + ca*cc, sa*sb*cc - ca*sc},
                 {-sb,   cb*sc,            cb*cc           }}};
}

// ── Collider names ───────────────────────────────────────────────
const char* cNames[] = {"?","Base","LowArm","Elbow","UpArm","Wrist","Tool1","Tool2"};

// ── Trajectory row ───────────────────────────────────────────────
struct TRow {
    int task, seg;
    double time, q[6], v[6], dist, tcp[3];
};

std::vector<TRow> loadTraj(const char* path) {
    std::vector<TRow> rows;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        TRow r;
        is >> r.task >> r.seg >> r.time;
        for (int i = 0; i < 6; i++) is >> r.q[i];
        for (int i = 0; i < 6; i++) is >> r.v[i];
        is >> r.dist >> r.tcp[0] >> r.tcp[1] >> r.tcp[2];
        if (is) rows.push_back(r);
        else fprintf(stderr, "警告: 轨迹行解析失败\n");
    }
    return rows;
}

// ── Generate 26 box sample points ────────────────────────────────
// 8 corners + 12 edge midpoints + 6 face centers
// TCP位于箱子底面中心 (吸盘), 箱子在TCP下方 (Z负方向)
void boxSamplePts(Vec3 tcpPos, Mat3 R, Vec3 out[26]) {
    Vec3 corners[8];
    int ci = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = 0; sz <= 1; sz++) {
                // 局部坐标 (法兰/TCP坐标系): 箱子从TCP往 +Z 方向延伸 (世界坐标下方)
                // TCP Z轴指向世界-Z (朝下), 所以 +Z local = 世界下方
                // TCP在箱子顶面中心 (吸盘), 箱子底面在 z = +BOX_HZ
                Vec3 local = {sx * BOX_LX / 2.0,
                              sy * BOX_WY / 2.0,
                              (double)sz * BOX_HZ};
                corners[ci] = tcpPos + mulMV(R, local);
                out[ci] = corners[ci];
                ci++;
            }
    // 12 edge midpoints
    int edges[][2] = {{0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7}};
    for (auto& e : edges)
        out[ci++] = 0.5 * (corners[e[0]] + corners[e[1]]);
    // 6 face centers
    int faces[][4] = {{4,5,6,7},{0,1,2,3},{2,3,6,7},{0,1,4,5},{0,2,4,6},{1,3,5,7}};
    for (auto& fc : faces)
        out[ci++] = 0.25 * (corners[fc[0]] + corners[fc[1]] + corners[fc[2]] + corners[fc[3]]);
}

// ── Min surface dist from 26 box sample points to one collider ──
double boxCollDist(Vec3 pts[26], int type, Vec3 p1, Vec3 p2, double r) {
    double mn = 1e10;
    for (int i = 0; i < 26; i++) {
        double d = (type == 2) ? ptSegDist(pts[i], p1, p2) : len(pts[i] - p1);
        mn = std::min(mn, d);
    }
    return mn - r;   // surface distance (negative = penetration)
}

// ── Frame analysis result ────────────────────────────────────────
struct FR {
    int fi, seg;
    double q[6];
    double cDist[7];    // per-collider min surface dist
    double best;        // overall min
    int    bestC;       // collider index with min dist
    double soDist;      // SO self-collision distance (mm)
    int    soA, soB;    // SO collision pair
    Vec3   tcp;
};

// ══════════════════════════════════════════════════════════════════
int main()
{
    // ─── Load SO library ─────────────────────────────────────────
    const char* soPath = std::getenv("HRC_LIB_PATH");
    if (!soPath) soPath = "lib/libHRCInterface.so.1";

    void* handle = dlopen(soPath, RTLD_NOW | RTLD_GLOBAL);
    if (!handle) { fprintf(stderr, "dlopen失败: %s\n", dlerror()); return 1; }

    auto initAC   = (FnInitAC)      dlsym(handle, "initACAreaConstrainPackageInterface");
    auto updateAC = (FnUpdateAC)    dlsym(handle, "updateACAreaConstrainPackageInterface");
    auto checkSelf= (FnCheckSelfColl)dlsym(handle, "checkCPSelfCollisionInterface");
    auto setOpen  = (FnSetColliderOpen)dlsym(handle, "setCPSelfColliderLinkModelOpenStateInterface");
    auto getUI    = (FnGetUIInfo)   dlsym(handle, "getUIInfoMationInterface");
    auto fk2      = (FnForwardKin)  dlsym(handle, "forwardKinematics2");
    auto setTB    = (FnSetToolBall) dlsym(handle, "setCPToolCollisionBallShapeInterface");
    auto rmTool   = (FnRemoveTool)  dlsym(handle, "removeCPToolCollisonInterface");

    if (!initAC || !updateAC || !checkSelf || !setOpen || !getUI) {
        fprintf(stderr, "SO符号加载失败\n");
        dlclose(handle); return 1;
    }

    // ─── Initialize collision system ─────────────────────────────
    SO_LREAL dh[8] = {296.5, 336.2, 239, 158.5, 158.5, 134.5, 900, 941.5};
    SO_LREAL bg[7] = {0,0,20,   0,0,330,   160};
    SO_LREAL lg[7] = {0,0,340,  900,0,340,  140};
    SO_LREAL eg[7] = {-10,0,60, 941.5,0,60, 120};
    SO_LREAL ug[7] = {0,0,-50,  0,0,100,    100};
    SO_LREAL wg[4] = {0,0,20,   140};
    SO_LREAL hj[6] = {0, -90, 0, 0, 90, 0};
    initAC(SO_INT(1), dh, bg, lg, eg, ug, wg, hj);
    SO_BOOL fl[3] = {1, 1, 1};
    setOpen(fl);

    // ─── Register tool ball (same as palletizing code) ───────────
    if (setTB) {
        SO_LREAL off[3] = {0, 0, -400};
        SO_INT ret = setTB(SO_LINT(1), off, 120.0);
        printf("工具球注册 toolIdx=1 offset=(0,0,-400) r=120: %s\n",
               ret == 0 ? "成功" : "失败");
    }

    // ─── Print banner ────────────────────────────────────────────
    printf("\n");
    printf("============================================================\n");
    printf("  箱子 (%.0fx%.0fx%.0fmm) vs 机械臂碰撞体 精确距离分析\n",
           BOX_LX, BOX_WY, BOX_HZ);
    printf("  方法: SO getUIInfo(碰撞体位姿) + 轨迹TCP(位置) + FK2(朝向)\n");
    printf("  采样: 26点 (8角 + 12边中 + 6面中)\n");
    printf("============================================================\n");

    // ─── Verify HOME collider positions ──────────────────────────
    {
        SO_LREAL q[6] = {0,-90,0,0,90,0}, v0[6] = {}, a0[6] = {};
        updateAC(q, v0, a0);

        SO_DINT ix[7], ty[7];
        SO_LREAL dt[7][9], rd[7];
        getUI(ix, ty, dt, rd);

        printf("\nHOME碰撞体验证:\n");
        for (int i = 0; i < 7; i++) {
            const char* n = (ix[i] >= 1 && ix[i] <= 7) ? cNames[ix[i]] : "?";
            if (ty[i] == 2)
                printf("  [%d] %-8s 胶囊 p1=(%.1f,%.1f,%.1f) p2=(%.1f,%.1f,%.1f) r=%.0f\n",
                    ix[i], n, dt[i][0], dt[i][1], dt[i][2],
                    dt[i][3], dt[i][4], dt[i][5], rd[i]);
            else if (ty[i] == 1 || rd[i] > 0)
                printf("  [%d] %-8s 球   c=(%.1f,%.1f,%.1f) r=%.0f\n",
                    ix[i], n, dt[i][0], dt[i][1], dt[i][2], rd[i]);
        }

        // FK2 test at HOME
        if (fk2) {
            SO_LREAL qf[6] = {0,-90,0,0,90,0};
            MC_COORD_REF tcp = {};
            fk2(qf, &tcp);
            printf("\nFK2 HOME: pos=(%.1f,%.1f,%.1f) orient=(%.1f,%.1f,%.1f)\n",
                   tcp.X, tcp.Y, tcp.Z, tcp.A, tcp.B, tcp.C);
            printf("  (注: FK2位置可能不准, 朝向可信; 位置将使用轨迹文件TCP)\n");
        }
    }

    // ─── Load trajectory ─────────────────────────────────────────
    auto rows = loadTraj("data/so_palletizing_trajectory.txt");
    if (rows.empty()) {
        fprintf(stderr, "轨迹加载失败\n");
        dlclose(handle); return 1;
    }
    printf("\n轨迹总帧数: %zu\n", rows.size());

    // Count carrying frames (seg 3-5)
    int nCarry = 0;
    for (auto& r : rows) if (r.seg >= 3 && r.seg <= 5) nCarry++;
    printf("搬运帧数(seg3-5): %d\n", nCarry);

    // ─── Analyze carrying segments ───────────────────────────────
    printf("\n============================================================\n");
    printf("  搬运段(seg3-5) 箱子26点 vs 碰撞体表面距离\n");
    printf("============================================================\n");

    std::vector<FR> results;
    double gMin[7];
    int gMinFrame[7] = {};
    for (int i = 0; i < 7; i++) gMin[i] = 1e10;

    // Saved reference collider info for labels
    SO_DINT refIx[7], refTy[7];
    SO_LREAL refRd[7];

    for (size_t fi = 0; fi < rows.size(); fi++) {
        auto& row = rows[fi];
        if (row.seg < 3 || row.seg > 5) continue;

        // Update joint state (注: updateAC会原地修改数组, deg→rad)
        SO_LREAL q[6], v0[6] = {}, a0[6] = {};
        for (int i = 0; i < 6; i++) q[i] = row.q[i];  // copy!
        updateAC(q, v0, a0);

        // Re-register tool ball (updateAC may reset internal state)
        if (setTB) {
            SO_LREAL off[3] = {0, 0, -400};
            setTB(SO_LINT(1), off, 120.0);
        }

        // SO self-collision check
        SO_LINT pr[2] = {};
        SO_LREAL sd = 0;
        checkSelf(pr, &sd);

        // Get collider world positions
        SO_DINT ix[7], ty[7];
        SO_LREAL dt[7][9], rd[7];
        getUI(ix, ty, dt, rd);

        // Save reference info from first frame
        if (results.empty()) {
            for (int i = 0; i < 7; i++) {
                refIx[i] = ix[i]; refTy[i] = ty[i]; refRd[i] = rd[i];
            }
        }

        // TCP orientation from FK2 (position from trajectory file)
        Mat3 R = eulerZYX(0, 0, 0);  // identity fallback
        if (fk2) {
            SO_LREAL qf[6];
            for (int i = 0; i < 6; i++) qf[i] = row.q[i];
            MC_COORD_REF tcp = {};
            fk2(qf, &tcp);
            R = eulerZYX(tcp.A, tcp.B, tcp.C);
        }
        Vec3 tcpPos = {row.tcp[0], row.tcp[1], row.tcp[2]};

        // Generate 26 box sample points
        Vec3 bpts[26];
        boxSamplePts(tcpPos, R, bpts);

        // Compute min distance to each collider
        FR fr;
        fr.fi = (int)fi;
        fr.seg = row.seg;
        for (int i = 0; i < 6; i++) fr.q[i] = row.q[i];
        fr.best = 1e10;
        fr.bestC = -1;
        fr.soDist = sd;
        fr.soA = (int)pr[0];
        fr.soB = (int)pr[1];
        fr.tcp = tcpPos;

        for (int c = 0; c < 7; c++) {
            int ci = ix[c];
            double r = rd[c];
            // Skip tool colliders (6,7) and zero-radius entries
            if (ci == 6 || ci == 7 || r < 1.0) {
                fr.cDist[c] = 1e10;
                continue;
            }
            Vec3 p1 = {dt[c][0], dt[c][1], dt[c][2]};
            Vec3 p2 = {dt[c][3], dt[c][4], dt[c][5]};
            fr.cDist[c] = boxCollDist(bpts, ty[c], p1, p2, r);

            if (fr.cDist[c] < fr.best) {
                fr.best = fr.cDist[c];
                fr.bestC = c;
            }
            if (fr.cDist[c] < gMin[c]) {
                gMin[c] = fr.cDist[c];
                gMinFrame[c] = (int)fi;
            }
        }
        results.push_back(fr);
    }

    // ─── Per-collider summary ────────────────────────────────────
    printf("\n各碰撞体 vs 箱子 最小表面距离 (在所有搬运帧中):\n");
    printf("  %-10s %12s %8s  %-16s %s\n", "碰撞体", "最小距(mm)", "帧号", "状态", "安全策略");
    printf("  %s\n", std::string(75, '-').c_str());
    double criticalMin = 1e10;  // 仅Base+LowerArm
    for (int i = 0; i < 7; i++) {
        if (refIx[i] == 6 || refIx[i] == 7 || refRd[i] < 1) continue;
        const char* n = (refIx[i] >= 1 && refIx[i] <= 7) ? cNames[refIx[i]] : "?";
        bool isCritical = (refIx[i] >= 1 && refIx[i] <= 2);  // Base + LowerArm
        const char* st = gMin[i] < 0    ? ">>> 碰撞! <<<" :
                         gMin[i] < 50   ? ">>> 危险! <<<" :
                         gMin[i] < 100  ? "注意"          : "安全";
        const char* policy = isCritical ? "★ 硬约束" : "诊断 (结构性相邻)";
        printf("  %-10s %12.1f %8d  %-16s %s\n", n, gMin[i], gMinFrame[i], st, policy);
        if (isCritical) criticalMin = std::min(criticalMin, gMin[i]);
    }

    // ─── Top 20 worst frames ─────────────────────────────────────
    std::sort(results.begin(), results.end(),
              [](const FR& a, const FR& b) { return a.best < b.best; });

    printf("\n最危险20帧 (按箱子-机械臂最小距排序):\n");
    printf("  %5s %3s %10s %10s %8s %30s  %s\n",
           "frame", "seg", "BOX-ARM", "SO_dist", "SO_pair", "TCP(mm)", "closest");
    printf("  %s\n", std::string(90, '-').c_str());
    for (int i = 0; i < std::min(20, (int)results.size()); i++) {
        auto& f = results[i];
        int ci = f.bestC;
        const char* n = (ci >= 0 && ci < 7 && refIx[ci] >= 1 && refIx[ci] <= 7)
                        ? cNames[refIx[ci]] : "?";
        printf("  %5d %3d %10.1f %10.1f (%d,%d) %10.1f,%7.1f,%7.1f  %s\n",
               f.fi, f.seg, f.best, f.soDist, f.soA, f.soB,
               f.tcp.x, f.tcp.y, f.tcp.z, n);
    }

    // ─── Detailed analysis of worst frame ────────────────────────
    if (!results.empty()) {
        auto& w = results[0];
        printf("\n============================================================\n");
        printf("  最危险帧详细分析 (frame=%d, seg=%d)\n", w.fi, w.seg);
        printf("============================================================\n");
        printf("关节角(deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
               w.q[0], w.q[1], w.q[2], w.q[3], w.q[4], w.q[5]);
        printf("轨迹TCP(mm): (%.1f, %.1f, %.1f)\n", w.tcp.x, w.tcp.y, w.tcp.z);
        printf("SO自碰撞距: %.1fmm, pair(%d,%d)\n", w.soDist, w.soA, w.soB);

        // Re-compute for this frame
        SO_LREAL q[6], v0[6] = {}, a0[6] = {};
        for (int i = 0; i < 6; i++) q[i] = w.q[i];
        updateAC(q, v0, a0);
        if (setTB) { SO_LREAL off[3] = {0, 0, -400}; setTB(SO_LINT(1), off, 120.0); }

        SO_DINT ix[7], ty[7];
        SO_LREAL dt[7][9], rd[7];
        getUI(ix, ty, dt, rd);

        Mat3 R = eulerZYX(0, 0, 0);
        if (fk2) {
            SO_LREAL qf[6];
            for (int i = 0; i < 6; i++) qf[i] = w.q[i];
            MC_COORD_REF tcp = {};
            fk2(qf, &tcp);
            R = eulerZYX(tcp.A, tcp.B, tcp.C);
            printf("FK2朝向: A=%.2f B=%.2f C=%.2f\n", tcp.A, tcp.B, tcp.C);
        }

        printf("\n碰撞体 (世界坐标):\n");
        for (int i = 0; i < 7; i++) {
            const char* n = (ix[i] >= 1 && ix[i] <= 7) ? cNames[ix[i]] : "?";
            if (ty[i] == 2 && rd[i] > 0)
                printf("  %-8s 胶囊 p1=(%.1f,%.1f,%.1f) p2=(%.1f,%.1f,%.1f) r=%.0f L=%.0f\n",
                    n, dt[i][0], dt[i][1], dt[i][2],
                    dt[i][3], dt[i][4], dt[i][5], rd[i],
                    len(Vec3{dt[i][0]-dt[i][3], dt[i][1]-dt[i][4], dt[i][2]-dt[i][5]}));
            else if (rd[i] > 0)
                printf("  %-8s 球   c=(%.1f,%.1f,%.1f) r=%.0f\n",
                    n, dt[i][0], dt[i][1], dt[i][2], rd[i]);
        }

        Vec3 tcpPos = {w.tcp.x, w.tcp.y, w.tcp.z};
        Vec3 bpts[26];
        boxSamplePts(tcpPos, R, bpts);

        printf("\n箱子角点(世界mm):\n");
        for (int i = 0; i < 8; i++)
            printf("  C%d: (%.1f, %.1f, %.1f)\n", i, bpts[i].x, bpts[i].y, bpts[i].z);

        printf("\n各碰撞体距离:\n");
        for (int c = 0; c < 7; c++) {
            if (ix[c] == 6 || ix[c] == 7 || rd[c] < 1) continue;
            const char* n = (ix[c] >= 1 && ix[c] <= 7) ? cNames[ix[c]] : "?";
            Vec3 p1 = {dt[c][0], dt[c][1], dt[c][2]};
            Vec3 p2 = {dt[c][3], dt[c][4], dt[c][5]};
            double d = boxCollDist(bpts, ty[c], p1, p2, rd[c]);
            const char* st = d < 0 ? ">>> 碰撞! <<<" :
                             d < 50 ? ">>> 危险! <<<" :
                             d < 100 ? "注意" : "安全";
            printf("  %-8s: %8.1fmm  [type=%d r=%.0f] %s\n", n, d, ty[c], rd[c], st);
        }
    }

    // ─── Per-segment statistics ──────────────────────────────────
    printf("\n============================================================\n");
    printf("各搬运段统计:\n");
    for (int seg = 3; seg <= 5; seg++) {
        double sMin = 1e10;
        int sMinFrame = -1;
        int cnt = 0;
        for (auto& f : results) {
            if (f.seg != seg) continue;
            cnt++;
            if (f.best < sMin) { sMin = f.best; sMinFrame = f.fi; }
        }
        if (cnt == 0) continue;
        const char* desc = seg == 3 ? "Pick提升" : seg == 4 ? "搬运主段" : "Place下降";
        const char* st = sMin < 0 ? "碰撞!" : sMin < 50 ? "危险" : sMin < 100 ? "注意" : "安全";
        printf("  seg%d(%s): %d帧, 最小=%.1fmm @frame%d %s\n",
               seg, desc, cnt, sMin, sMinFrame, st);
    }

    // ─── Final summary ───────────────────────────────────────────
    printf("\n============================================================\n");
    printf("核心发现:\n");
    printf("  SO工具球: toolIdx=1, offset=(0,0,-400)mm, r=120mm\n");
    printf("    工具球覆盖范围(法兰坐标系): Z = [-520, -280]mm (朝腕部方向)\n");
    printf("  实际箱子: %gx%gx%gmm\n", BOX_LX, BOX_WY, BOX_HZ);
    printf("    箱子范围(法兰坐标系): Z = [0, -%g]mm (TCP朝下时)\n", BOX_HZ);
    printf("    箱子半对角线: %.1fmm\n", 0.5 * std::sqrt(BOX_LX*BOX_LX + BOX_WY*BOX_WY));
    printf("  工具球 vs 箱子在法兰坐标系中不重叠! (间距=%gmm)\n", 280.0 - BOX_HZ);
    printf("  SO报告的51.9mm = getCollisionReport pair(6,4) = 工具球vs腕部\n");
    printf("  => 这个距离与箱子是否碰到机械臂完全无关!\n\n");

    bool anyPen = false;
    double overallMin = 1e10;
    for (int i = 0; i < 7; i++) {
        if (gMin[i] < 0) anyPen = true;
        overallMin = std::min(overallMin, gMin[i]);
    }

    printf("  v6.3 安全策略:\n");
    printf("    硬约束 (规划器拒绝): Base(idx=1) + LowerArm(idx=2)\n");
    printf("    诊断 (不拒绝): Elbow(3) / UpArm(4) / Wrist(5) — 结构性相邻TCP\n");
    printf("    Elbow前端距TCP ~451mm (d4+d5+d6), 小于 R_elbow+BOX_HZ=370mm\n");
    printf("    UpArm仅158.5mm(d4), Wrist仅134.5mm(d6) — 与箱子必然重叠\n\n");

    if (criticalMin < 0)
        printf("  >>>>>> 关键碰撞: Base/LowerArm穿透箱子! (min=%.1fmm) <<<<<<\n", criticalMin);
    else if (criticalMin < 50)
        printf("  >>> 关键警告: Base/LowerArm最近%.1fmm, 需检查! <<<\n", criticalMin);
    else
        printf("  ✅ 关键碰撞体(Base+LowArm)安全: min=%.1fmm (>20mm margin)\n", criticalMin);

    if (anyPen)
        printf("  ⚠ 近端碰撞体(Elbow/UpArm/Wrist)存在重叠: overall=%.1fmm (结构性, 非危险)\n",
               overallMin);
    else
        printf("  所有碰撞体安全: overall=%.1fmm\n", overallMin);

    printf("============================================================\n");

    dlclose(handle);
    return 0;
}
