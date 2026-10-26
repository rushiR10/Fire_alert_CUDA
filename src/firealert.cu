// ============================================================================
// firealert.cu  -  Hospital fire-alert server using Criticality- and
//                  Deadline-Aware Graph Burning (CDA-GB), simulation.
//
//   A fire sensor reports a room to the server. Every round the server can
//   push the alert DIRECTLY to one phone (a graph-burning source); every phone
//   that already has the alert RELAYS it to phones it is in contact with in
//   that round (Bluetooth / Wi-Fi Direct). The server picks the next source
//   with the CDA-GB score; the candidate scoring runs on the GPU (one CUDA
//   block per candidate phone) and falls back to the CPU when there is no GPU.
//
//   Modes:  demo    one terminal, full animation, no networking
//           server  TCP server: phones connect, a sensor reports the fire
//           phone   simulated phone(s) that receive alerts from the server
//           sensor  fire sensor: sends "FIRE <room>" to the server
//
//   Build:  nvcc -O3 -std=c++17 -o bin/firealert src/firealert.cu        (GPU)
//           c++ -O2 -std=c++17 -pthread -x c++ src/firealert.cu -o bin/firealert   (CPU)
// ============================================================================
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET sock_t;
#define CLOSESOCK closesocket
#define BAD_SOCK INVALID_SOCKET
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define CLOSESOCK close
#define BAD_SOCK (-1)
#endif
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#ifdef __CUDACC__
#include <cuda_runtime.h>
#endif

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static const int INF8 = 255;  // "never informed"
typedef std::chrono::steady_clock Clock;
static double msSince(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
static void sleepMs(int ms) {
    if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ----------------------------------------------------------------------------
// Options
// ----------------------------------------------------------------------------
struct Opt {
    std::string mode = "help";
    int people = 40, H = 10, K = 4, port = 9000, delayMs = 1500, roundSec = 30;
    uint64_t seed = 7;
    double w = 0.3, fail = 0.0;
    bool cpu = false, verify = true, clear = true, color = true;
    std::string fire = "ICU", host = "127.0.0.1", id, room;
    std::string exportPath;  // --export: write the browser replay (viewer/replay_data.js)
};
static Opt g_opt;

// ----------------------------------------------------------------------------
// Terminal colours
// ----------------------------------------------------------------------------
static const char* RESET = "\033[0m";
static const char* BOLD = "\033[1m";
static const char* C_SRC = "\033[1;97;41m";  // direct push from server
static const char* C_ON = "\033[1;32m";      // informed before deadline
static const char* C_LATE = "\033[33m";      // informed after deadline
static const char* C_WAIT = "\033[1;35m";    // critical, not informed, deadline not passed
static const char* C_MISS = "\033[31m";      // critical, deadline passed, not informed
static const char* C_IDLE = "\033[90m";      // not critical, not informed
static const char* C_FIRE = "\033[1;97;41m";
static const char* C_NEAR = "\033[1;33m";
static const char* C_CYAN = "\033[1;36m";
static std::string col(const char* c) { return g_opt.color ? std::string(c) : std::string(); }
static std::string paint(const std::string& s, const char* c) { return col(c) + s + col(RESET); }

// ----------------------------------------------------------------------------
// Deterministic RNG (same results on every OS / compiler)
// ----------------------------------------------------------------------------
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0x632BE59BD9B4E019ULL) {}
    uint64_t next() {
        uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    double u() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
    int below(int n) { return (int)(next() % (uint64_t)n); }
};

// ----------------------------------------------------------------------------
// Hospital floor: 3 x 4 grid of rooms, neighbours share a wall / door
// ----------------------------------------------------------------------------
static const int ROWS = 3, COLS = 4, NROOMS = ROWS * COLS;
enum { RM_ICU, RM_ER, RM_OT, RM_RADIOLOGY, RM_WARD_A, RM_NURSE_STN, RM_WARD_B, RM_LAB, RM_PHARMACY, RM_LOBBY, RM_ADMIN_OFF, RM_CANTEEN };
static const char* ROOM_NAME[NROOMS] = {"ICU",    "ER",       "OT",     "Radiology", "Ward-A", "NurseStn",
                                        "Ward-B", "Lab",      "Pharmacy", "Lobby",   "Admin",  "Canteen"};

static int roomDist(int a, int b) { return abs(a / COLS - b / COLS) + abs(a % COLS - b % COLS); }
static std::vector<int> roomNeighbours(int a) {
    std::vector<int> nb;
    for (int b = 0; b < NROOMS; ++b)
        if (roomDist(a, b) == 1) nb.push_back(b);
    return nb;
}
static std::string lower(std::string s) {
    for (auto& ch : s) ch = (char)tolower((unsigned char)ch);
    return s;
}
static int findRoom(const std::string& s) {
    for (int a = 0; a < NROOMS; ++a)
        if (lower(s) == lower(ROOM_NAME[a])) return a;
    if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit)) {
        int a = atoi(s.c_str());
        if (a >= 0 && a < NROOMS) return a;
    }
    return -1;
}

// ----------------------------------------------------------------------------
// People. Criticality per role follows Section 7.2 of the proposal.
// ----------------------------------------------------------------------------
enum Role { R_COORD, R_DOCTOR, R_NURSE, R_PATIENT, R_ADMIN, R_VISITOR, NROLES };
struct RoleInfo {
    const char* name;
    char tag;
    int crit;       // base criticality (1..5)
    double move;    // probability of moving to a neighbouring room per round
    double frac;    // share of the population
    std::vector<int> homes;
    const char* action;
};
static const RoleInfo ROLES[NROLES] = {
    {"Coordinator", 'C', 5, 0.30, 0.05, {RM_NURSE_STN, RM_ADMIN_OFF}, "Activate the fire plan, call the fire brigade, direct the evacuation."},
    {"Doctor", 'D', 5, 0.35, 0.15, {RM_ICU, RM_ER, RM_OT, RM_WARD_A, RM_WARD_B, RM_RADIOLOGY}, "Move critical patients out of danger and coordinate with the nurses."},
    {"Nurse", 'N', 4, 0.35, 0.25, {RM_ICU, RM_ER, RM_WARD_A, RM_WARD_B, RM_NURSE_STN, RM_OT, RM_LAB}, "Evacuate the patients in your area and close the fire doors."},
    {"Patient", 'P', 3, 0.05, 0.35, {RM_ICU, RM_WARD_A, RM_WARD_B, RM_ER}, "Stay calm and follow the staff to the nearest safe exit."},
    {"Admin", 'A', 2, 0.15, 0.10, {RM_ADMIN_OFF, RM_PHARMACY, RM_LOBBY, RM_LAB}, "Open the exits and guide people to the assembly point."},
    {"Visitor", 'V', 1, 0.25, 0.10, {RM_LOBBY, RM_CANTEEN, RM_WARD_A, RM_WARD_B}, "Leave the building by the nearest safe exit."},
};

struct Person {
    std::string name;
    int role;
};
struct Hospital {
    int N = 0;
    std::vector<Person> P;
    std::vector<int> room0;  // where everybody is when the fire starts
};

static Hospital buildHospital(int N, uint64_t seed) {
    Hospital h;
    Rng rng(seed);
    int cnt[NROLES], used = 0;
    for (int r = 0; r < NROLES; ++r) {
        if (r == R_PATIENT) continue;
        cnt[r] = std::max(1, (int)(ROLES[r].frac * N + 0.5));
        used += cnt[r];
    }
    cnt[R_PATIENT] = N - used;
    for (int r = 0; r < NROLES; ++r) {
        for (int i = 1; i <= cnt[r]; ++i) {
            char buf[16];
            snprintf(buf, sizeof buf, "%c%02d", ROLES[r].tag, i);
            int home = ROLES[r].homes[rng.below((int)ROLES[r].homes.size())];
            if (rng.u() < ROLES[r].move) {
                auto nb = roomNeighbours(home);
                home = nb[rng.below((int)nb.size())];
            }
            h.P.push_back({buf, r});
            h.room0.push_back(home);
        }
    }
    h.N = (int)h.P.size();
    return h;
}
static int findPerson(const Hospital& h, const std::string& s) {
    for (int v = 0; v < h.N; ++v)
        if (lower(s) == lower(h.P[v].name)) return v;
    if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit)) {
        int v = atoi(s.c_str());
        if (v >= 0 && v < h.N) return v;
    }
    return -1;
}

// ----------------------------------------------------------------------------
// Temporal contact graph: edges of round r are e[off[r] .. off[r+1])
// ----------------------------------------------------------------------------
struct Edge {
    int x, y;
};
struct TGraph {
    int H = 0;
    std::vector<int> off;
    std::vector<Edge> e;
};

struct Scenario {
    int N = 0, H = 0, fire = 0, critTotal = 0;
    std::vector<std::vector<int>> room;  // room[r][v], r = 0..H
    std::vector<int> crit, dl;
    std::vector<char> critical;
    TGraph plan;  // contacts the server predicts (indoor positioning / history)
    TGraph exec;  // contacts that really work (plan minus unknown link failures)
};

// Step 1-3 of the proposal: temporal graph, fire-aware criticality, deadlines
static Scenario makeScenario(const Hospital& h, int fire, int H, double fail, uint64_t seed) {
    Scenario S;
    S.N = h.N;
    S.H = H;
    S.fire = fire;
    Rng rng(seed), rngFail(seed ^ 0xF00DULL);
    S.room.assign(H + 1, std::vector<int>(h.N));
    S.room[0] = h.room0;
    for (int r = 1; r <= H; ++r)
        for (int v = 0; v < h.N; ++v) {
            int a = S.room[r - 1][v];
            if (rng.u() < ROLES[h.P[v].role].move) {
                auto nb = roomNeighbours(a);
                a = nb[rng.below((int)nb.size())];
            }
            S.room[r][v] = a;
        }

    // criticality = role criticality + closeness to the fire, deadline = 7 - criticality
    S.crit.resize(h.N);
    S.dl.resize(h.N);
    S.critical.resize(h.N);
    for (int v = 0; v < h.N; ++v) {
        int d = roomDist(S.room[0][v], fire);
        int bonus = d == 0 ? 2 : d == 1 ? 1 : 0;
        S.crit[v] = std::min(5, ROLES[h.P[v].role].crit + bonus);
        S.dl[v] = 7 - S.crit[v];
        S.critical[v] = S.crit[v] >= 4;
        S.critTotal += S.critical[v];
    }

    // device-to-device contacts: ~2 contacts per person per round inside a room,
    // a few through the walls of neighbouring rooms
    auto pSame = [](size_t n) { return n < 2 ? 0.6 : std::min(0.6, 2.0 / (double)(n - 1)); };
    S.plan.H = S.exec.H = H;
    S.plan.off.assign(H + 2, 0);
    S.exec.off.assign(H + 2, 0);
    for (int r = 1; r <= H; ++r) {
        S.plan.off[r] = (int)S.plan.e.size();
        S.exec.off[r] = (int)S.exec.e.size();
        std::vector<std::vector<int>> occ(NROOMS);
        for (int v = 0; v < h.N; ++v) occ[S.room[r][v]].push_back(v);
        std::vector<Edge> es;
        for (int a = 0; a < NROOMS; ++a) {
            double p = pSame(occ[a].size());
            for (size_t i = 0; i < occ[a].size(); ++i)
                for (size_t j = i + 1; j < occ[a].size(); ++j)
                    if (rng.u() < p) es.push_back({occ[a][i], occ[a][j]});
            for (int b : roomNeighbours(a)) {
                if (b < a) continue;
                double q = 0.1 * std::min(pSame(occ[a].size()), pSame(occ[b].size()));
                for (int x : occ[a])
                    for (int y : occ[b])
                        if (rng.u() < q) es.push_back({x, y});
            }
        }
        for (auto& ed : es) {
            S.plan.e.push_back(ed);
            if (rngFail.u() >= fail) S.exec.e.push_back(ed);
        }
    }
    S.plan.off[H + 1] = (int)S.plan.e.size();
    S.exec.off[H + 1] = (int)S.exec.e.size();
    return S;
}

// ----------------------------------------------------------------------------
// Propagation (Section 7.4). rdy[v] = first round in which v can transmit.
//   round k: the source of round k is informed and transmits in round k;
//            a node reached in round r has arrival r and transmits from r+1.
// ----------------------------------------------------------------------------
static void propagate(const TGraph& g, int kFrom, int kTo, std::vector<uint8_t>& rdy,
                      std::vector<int>* parent = nullptr) {
    for (int r = kFrom; r <= kTo; ++r)
        for (int i = g.off[r]; i < g.off[r + 1]; ++i) {
            int u = g.e[i].x, v = g.e[i].y;
            if (rdy[u] <= r && rdy[v] == INF8) {
                rdy[v] = (uint8_t)(r + 1);
                if (parent) (*parent)[v] = u;
            } else if (rdy[v] <= r && rdy[u] == INF8) {
                rdy[u] = (uint8_t)(r + 1);
                if (parent) (*parent)[u] = v;
            }
        }
}

// One real round: activate the source, then one hop over the links of G(k)
static void realRound(const TGraph& g, int k, int src, std::vector<uint8_t>& arr, std::vector<int>& parent) {
    const int N = (int)arr.size();
    if (src >= 0 && arr[src] == INF8) {
        arr[src] = (uint8_t)k;
        parent[src] = -1;  // -1 = server
    }
    std::vector<uint8_t> rdy(N);
    for (int v = 0; v < N; ++v) rdy[v] = arr[v] != INF8 ? (uint8_t)k : (uint8_t)INF8;
    propagate(g, k, k, rdy, &parent);
    for (int v = 0; v < N; ++v)
        if (arr[v] == INF8 && rdy[v] == k + 1) arr[v] = (uint8_t)k;
}

// Where the alert will get with no new source (used for AlreadyCovered / Timely)
static void project(const TGraph& g, int k, int H, const std::vector<uint8_t>& arr, std::vector<uint8_t>& proj,
                    std::vector<uint8_t>& rdy) {
    const int N = (int)arr.size();
    for (int v = 0; v < N; ++v) rdy[v] = arr[v] != INF8 ? (uint8_t)k : (uint8_t)INF8;
    propagate(g, k, H, rdy);
    for (int v = 0; v < N; ++v)
        proj[v] = arr[v] != INF8 ? arr[v] : (rdy[v] == INF8 ? (uint8_t)INF8 : (uint8_t)(rdy[v] - 1));
}

// ----------------------------------------------------------------------------
// CDA-GB candidate score (Steps 4-7).
//   Gain    = sum over newly + timely reached v of crit(v) / (1 + ArrivalDelay)
//   Overlap = |reach(u) & AlreadyCovered| / |reach(u)|
//   Score   = Gain * (1 - w * Overlap)
// Gain is accumulated as an integer scaled by L = lcm(1..H), so the GPU and
// the CPU produce exactly the same numbers in any summation order.
// ----------------------------------------------------------------------------
struct CandStats {
    long long gain = -1;
    int reach = 0, ov = 0, nw = 0;
    bool operator==(const CandStats& o) const {
        return gain == o.gain && reach == o.reach && ov == o.ov && nw == o.nw;
    }
};

static long long lcmUpTo(int H) {
    long long l = 1;
    for (int i = 2; i <= H; ++i) l = l / std::gcd(l, (long long)i) * i;
    return l;
}
static double finalScore(const CandStats& s, double w, long long L) {
    double gain = (double)s.gain / (double)L;
    double ovr = s.reach > 0 ? (double)s.ov / (double)s.reach : 0.0;
    return gain * (1.0 - w * ovr);
}

static CandStats scoreCandCPU(const Scenario& S, int k, int c, long long L, const std::vector<uint8_t>& arr,
                              const std::vector<uint8_t>& proj, std::vector<uint8_t>& rdy) {
    std::fill(rdy.begin(), rdy.end(), (uint8_t)INF8);
    rdy[c] = (uint8_t)k;  // temporary simulation: the real state is not touched
    propagate(S.plan, k, S.H, rdy);
    CandStats st;
    st.gain = 0;
    for (int v = 0; v < S.N; ++v) {
        if (rdy[v] == INF8) continue;
        ++st.reach;
        int arrC = (v == c) ? k : rdy[v] - 1;
        uint8_t pa = proj[v];
        if (pa != INF8) ++st.ov; else ++st.nw;
        bool timelyByCurrent = (pa != INF8 && pa <= S.dl[v]);
        if (arr[v] == INF8 && !timelyByCurrent && arrC <= S.dl[v]) st.gain += (long long)S.crit[v] * (L / arrC);
    }
    return st;
}

static bool better(double s, int n, int c, double bs, int bn, int best) {
    if (best < 0) return true;
    if (s != bs) return s > bs;
    if (n != bn) return n > bn;
    return c < best;
}

// ----------------------------------------------------------------------------
// CUDA: one thread block per candidate phone. The block keeps the ready-round
// array of every phone in shared memory; its threads sweep each round's
// contacts in parallel, then reduce gain / reach / overlap.
// ----------------------------------------------------------------------------
#ifdef __CUDACC__
#define CUDA_CHECK(x)                                                                                    \
    do {                                                                                                 \
        cudaError_t e_ = (x);                                                                            \
        if (e_ != cudaSuccess) {                                                                         \
            fprintf(stderr, "CUDA error: %s (%s:%d)\n", cudaGetErrorString(e_), __FILE__, __LINE__);     \
            exit(1);                                                                                     \
        }                                                                                                \
    } while (0)

__global__ void scoreKernel(int k, int H, int N, long long L, const int* __restrict__ off,
                            const Edge* __restrict__ edges, const uint8_t* __restrict__ arr,
                            const uint8_t* __restrict__ proj, const int* __restrict__ crit,
                            const int* __restrict__ dl, long long* outGain, int* outReach, int* outOv,
                            int* outNew) {
    const int c = blockIdx.x;
    if (arr[c] != INF8) {  // already informed: not a candidate
        if (threadIdx.x == 0) { outGain[c] = -1; outReach[c] = outOv[c] = outNew[c] = 0; }
        return;
    }
    extern __shared__ uint8_t rdy[];
    __shared__ unsigned long long sGain;
    __shared__ int sReach, sOv, sNew;
    for (int v = threadIdx.x; v < N; v += blockDim.x) rdy[v] = INF8;
    if (threadIdx.x == 0) { sGain = 0ULL; sReach = sOv = sNew = 0; }
    __syncthreads();
    if (threadIdx.x == 0) rdy[c] = (uint8_t)k;
    __syncthreads();

    // temporal reachability: one hop per round; concurrent writers only write r+1,
    // so the result does not depend on thread order
    for (int r = k; r <= H; ++r) {
        for (int i = off[r] + threadIdx.x; i < off[r + 1]; i += blockDim.x) {
            Edge ed = edges[i];
            uint8_t ru = rdy[ed.x], rv = rdy[ed.y];
            if (ru <= r && rv == INF8) rdy[ed.y] = (uint8_t)(r + 1);
            else if (rv <= r && ru == INF8) rdy[ed.x] = (uint8_t)(r + 1);
        }
        __syncthreads();
    }

    long long g = 0;
    int reach = 0, ov = 0, nw = 0;
    for (int v = threadIdx.x; v < N; v += blockDim.x) {
        uint8_t rv = rdy[v];
        if (rv == INF8) continue;
        ++reach;
        int arrC = (v == c) ? k : rv - 1;
        uint8_t pa = proj[v];
        if (pa != INF8) ++ov; else ++nw;
        bool timelyByCurrent = (pa != INF8 && pa <= dl[v]);
        if (arr[v] == INF8 && !timelyByCurrent && arrC <= dl[v]) g += (long long)crit[v] * (L / arrC);
    }
    atomicAdd(&sGain, (unsigned long long)g);
    atomicAdd(&sReach, reach);
    atomicAdd(&sOv, ov);
    atomicAdd(&sNew, nw);
    __syncthreads();
    if (threadIdx.x == 0) {
        outGain[c] = (long long)sGain;
        outReach[c] = sReach;
        outOv[c] = sOv;
        outNew[c] = sNew;
    }
}

static bool gpuDetect(std::string& name) {
    int n = 0;
    if (cudaGetDeviceCount(&n) != cudaSuccess || n == 0) return false;
    cudaDeviceProp p;
    if (cudaGetDeviceProperties(&p, 0) != cudaSuccess) return false;
    name = p.name;
    return true;
}

struct GpuCtx {
    int N = 0;
    int *dOff = nullptr, *dCrit = nullptr, *dDl = nullptr, *dReach = nullptr, *dOv = nullptr, *dNew = nullptr;
    Edge* dE = nullptr;
    uint8_t *dArr = nullptr, *dProj = nullptr;
    long long* dGain = nullptr;
    cudaEvent_t e0, e1;
    bool live = false;

    void upload(const Scenario& S) {
        N = S.N;
        size_t ne = std::max<size_t>(1, S.plan.e.size());
        CUDA_CHECK(cudaMalloc(&dOff, S.plan.off.size() * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&dE, ne * sizeof(Edge)));
        CUDA_CHECK(cudaMalloc(&dCrit, N * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&dDl, N * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&dArr, N));
        CUDA_CHECK(cudaMalloc(&dProj, N));
        CUDA_CHECK(cudaMalloc(&dGain, N * sizeof(long long)));
        CUDA_CHECK(cudaMalloc(&dReach, N * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&dOv, N * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&dNew, N * sizeof(int)));
        CUDA_CHECK(cudaMemcpy(dOff, S.plan.off.data(), S.plan.off.size() * sizeof(int), cudaMemcpyHostToDevice));
        if (!S.plan.e.empty())
            CUDA_CHECK(cudaMemcpy(dE, S.plan.e.data(), S.plan.e.size() * sizeof(Edge), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(dCrit, S.crit.data(), N * sizeof(int), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(dDl, S.dl.data(), N * sizeof(int), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaEventCreate(&e0));
        CUDA_CHECK(cudaEventCreate(&e1));
        live = true;
    }
    void release() {
        if (!live) return;
        cudaFree(dOff); cudaFree(dE); cudaFree(dCrit); cudaFree(dDl); cudaFree(dArr); cudaFree(dProj);
        cudaFree(dGain); cudaFree(dReach); cudaFree(dOv); cudaFree(dNew);
        cudaEventDestroy(e0); cudaEventDestroy(e1);
        live = false;
    }
    // returns kernel time in ms
    float score(int k, int H, long long L, const std::vector<uint8_t>& arr, const std::vector<uint8_t>& proj,
                std::vector<CandStats>& out) {
        CUDA_CHECK(cudaMemcpy(dArr, arr.data(), N, cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(dProj, proj.data(), N, cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaEventRecord(e0));
        scoreKernel<<<N, 128, N>>>(k, H, N, L, dOff, dE, dArr, dProj, dCrit, dDl, dGain, dReach, dOv, dNew);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaEventRecord(e1));
        CUDA_CHECK(cudaEventSynchronize(e1));
        float ms = 0;
        CUDA_CHECK(cudaEventElapsedTime(&ms, e0, e1));
        std::vector<long long> g(N);
        std::vector<int> re(N), ov(N), nw(N);
        CUDA_CHECK(cudaMemcpy(g.data(), dGain, N * sizeof(long long), cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(re.data(), dReach, N * sizeof(int), cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(ov.data(), dOv, N * sizeof(int), cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(nw.data(), dNew, N * sizeof(int), cudaMemcpyDeviceToHost));
        out.assign(N, CandStats());
        for (int c = 0; c < N; ++c) out[c] = {g[c], re[c], ov[c], nw[c]};
        return ms;
    }
};
#endif

// ----------------------------------------------------------------------------
// Planner: picks the server's next direct push (Step 8) from the real state
// ----------------------------------------------------------------------------
struct Choice {
    int src = -1;
    double score = 0;
    CandStats st;
    std::string why;
    std::vector<std::pair<double, int>> top;  // best candidates, best first
};

struct Planner {
    const Scenario* S = nullptr;
    double w = 0.3;
    long long L = 1;
    bool useGpu = false, verify = true;
    std::string backend;
    // last call
    int cands = 0;
    double gpuMs = 0, kernelMs = 0, cpuMs = 0;
    bool match = true;
    // totals
    int calls = 0, mismatches = 0;
    double totGpuMs = 0, totCpuMs = 0;
#ifdef __CUDACC__
    GpuCtx G;
#endif

    void init(const Scenario& s, double w_, bool wantGpu, bool verify_) {
        S = &s;
        w = w_;
        L = lcmUpTo(s.H);
        verify = verify_;
        useGpu = false;
        backend = "CPU (built without CUDA)";
#ifdef __CUDACC__
        std::string name;
        if (!wantGpu) backend = "CPU (--cpu)";
        else if (!gpuDetect(name)) backend = "CPU (no CUDA GPU found)";
        else if (s.N > 40000) backend = "CPU (too many phones for shared memory)";
        else {
            useGpu = true;
            backend = "CUDA GPU " + name;
            G.upload(s);
        }
#else
        (void)wantGpu;
#endif
    }
    void release() {
#ifdef __CUDACC__
        G.release();
#endif
    }

    int choose(int k, const std::vector<uint8_t>& arr, Choice& ch) {
        ch = Choice();
        const int N = S->N;
        bool critLeft = false;
        for (int v = 0; v < N; ++v)
            if (S->critical[v] && arr[v] == INF8) critLeft = true;
        if (!critLeft) {  // Step 10: stop when every critical person has the alert
            ch.why = "every critical person already has the alert";
            return -1;
        }
        std::vector<uint8_t> proj(N), rdy(N);
        project(S->plan, k, S->H, arr, proj, rdy);
        cands = 0;
        for (int v = 0; v < N; ++v) cands += arr[v] == INF8;

        std::vector<CandStats> gst, cst(N);
#ifdef __CUDACC__
        if (useGpu) {
            auto t0 = Clock::now();
            kernelMs = G.score(k, S->H, L, arr, proj, gst);
            gpuMs = msSince(t0);
            totGpuMs += gpuMs;
        }
#endif
        if (!useGpu || verify) {
            auto t0 = Clock::now();
            for (int c = 0; c < N; ++c)
                if (arr[c] == INF8) cst[c] = scoreCandCPU(*S, k, c, L, arr, proj, rdy);
            cpuMs = msSince(t0);
            totCpuMs += cpuMs;
        }
        const std::vector<CandStats>& st = useGpu ? gst : cst;
        match = true;
        if (useGpu && verify)
            for (int c = 0; c < N; ++c)
                if (arr[c] == INF8 && !(gst[c] == cst[c])) match = false;
        mismatches += !match;
        ++calls;

        int best = -1, bn = -1;
        double bs = -1;
        std::vector<std::pair<double, int>> all;
        for (int c = 0; c < N; ++c) {
            if (arr[c] != INF8) continue;
            double s = finalScore(st[c], w, L);
            all.push_back({s, c});
            if (better(s, st[c].nw, c, bs, bn, best)) { bs = s; bn = st[c].nw; best = c; }
        }
        std::sort(all.begin(), all.end(), [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
            return a.first != b.first ? a.first > b.first : a.second < b.second;
        });
        for (size_t i = 0; i < all.size() && ch.top.size() < 4; ++i)
            if (all[i].second != best) ch.top.push_back(all[i]);
        if (best >= 0) { ch.src = best; ch.score = bs; ch.st = st[best]; }
        return best;
    }
};

// ----------------------------------------------------------------------------
// Metrics (Section 16)
// ----------------------------------------------------------------------------
struct Metrics {
    int critOn = 0, critTotal = 0, informed = 0, N = 0;
    double tcc = 0, wtc = 0, avgArrival = 0;
};
static Metrics evaluate(const Scenario& S, const std::vector<uint8_t>& arr) {
    Metrics m;
    m.N = S.N;
    double wOn = 0, wAll = 0, sumArr = 0;
    for (int v = 0; v < S.N; ++v) {
        bool on = arr[v] != INF8 && arr[v] <= S.dl[v];
        wAll += S.crit[v];
        if (on) wOn += S.crit[v];
        if (S.critical[v]) { ++m.critTotal; m.critOn += on; }
        if (arr[v] != INF8) { ++m.informed; sumArr += arr[v]; }
    }
    m.tcc = m.critTotal ? (double)m.critOn / m.critTotal : 0;
    m.wtc = wAll > 0 ? wOn / wAll : 0;
    m.avgArrival = m.informed ? sumArr / m.informed : 0;
    return m;
}

// Baselines from Section 15, run on the same network for the final comparison
static std::vector<uint8_t> runBaseline(const Scenario& S, int K, const std::string& kind, uint64_t seed) {
    const int N = S.N;
    std::vector<uint8_t> arr(N, INF8);
    std::vector<int> parent(N, -2);
    Rng rng(seed);
    for (int k = 1; k <= S.H; ++k) {
        int src = -1;
        if (k <= K) {
            std::vector<int> deg(N, 0), cand;
            for (int i = S.plan.off[k]; i < S.plan.off[k + 1]; ++i) { ++deg[S.plan.e[i].x]; ++deg[S.plan.e[i].y]; }
            for (int v = 0; v < N; ++v)
                if (arr[v] == INF8) cand.push_back(v);
            if (!cand.empty()) {
                if (kind == "random") src = cand[rng.below((int)cand.size())];
                else
                    for (int v : cand) {
                        if (src < 0) { src = v; continue; }
                        if (kind == "degree" && deg[v] > deg[src]) src = v;
                        if (kind == "crit" && (S.crit[v] > S.crit[src] || (S.crit[v] == S.crit[src] && deg[v] > deg[src])))
                            src = v;
                    }
            }
        }
        realRound(S.exec, k, src, arr, parent);
    }
    return arr;
}

// ----------------------------------------------------------------------------
// Terminal view
// ----------------------------------------------------------------------------
struct CellLine {
    std::string s;
    int vis = 0;
    void add(const std::string& txt, const char* c = nullptr) {
        s += c ? paint(txt, c) : txt;
        vis += (int)txt.size();
    }
};
static std::string repeat(const char* s, int n) {
    std::string r;
    for (int i = 0; i < n; ++i) r += s;
    return r;
}
static std::string fmtTime(int sec) {
    char b[24];
    snprintf(b, sizeof b, "t+%02d:%02d", sec / 60, sec % 60);
    return b;
}
static std::string pct(int a, int b) {
    char buf[64];
    snprintf(buf, sizeof buf, "%d/%d (%.0f%%)", a, b, b ? 100.0 * a / b : 0.0);
    return buf;
}

static const char* statusColor(const Scenario& S, const std::vector<uint8_t>& arr, const std::vector<char>& isSrc,
                               int v, int k) {
    if (isSrc[v]) return C_SRC;
    if (arr[v] != INF8) return arr[v] <= S.dl[v] ? C_ON : C_LATE;
    if (S.critical[v]) return k < S.dl[v] ? C_WAIT : C_MISS;
    return C_IDLE;
}

static void printMap(const Hospital& h, const Scenario& S, int k, const std::vector<uint8_t>& arr,
                     const std::vector<char>& isSrc, const std::vector<char>& isNew) {
    const int CW = 21;
    const bool tokens = S.N <= 60;
    const int snap = std::min(k, S.H);
    std::vector<std::vector<CellLine>> cell(NROOMS);
    for (int a = 0; a < NROOMS; ++a) {
        CellLine lab;
        int d = roomDist(a, S.fire);
        if (d == 0) lab.add(std::string(" FIRE ") + ROOM_NAME[a] + " ", C_FIRE);
        else if (d == 1) { lab.add(ROOM_NAME[a], BOLD); lab.add("  near fire", C_NEAR); }
        else lab.add(ROOM_NAME[a], BOLD);
        cell[a].push_back(lab);
        std::vector<int> in;
        for (int v = 0; v < S.N; ++v)
            if (S.room[snap][v] == a) in.push_back(v);
        if (tokens) {
            CellLine ln;
            for (int v : in) {
                const std::string& nm = h.P[v].name;
                if (ln.vis + 1 + (int)nm.size() > CW) { cell[a].push_back(ln); ln = CellLine(); }
                ln.add(" ");
                std::string c = col(statusColor(S, arr, isSrc, v, k));
                if (isNew[v]) c += col("\033[4m");
                ln.s += c + nm + col(RESET);
                ln.vis += (int)nm.size();
            }
            if (ln.vis > 0) cell[a].push_back(ln);
        } else {
            int on = 0, late = 0, wait = 0, miss = 0;
            for (int v : in) {
                const char* c = statusColor(S, arr, isSrc, v, k);
                on += (c == C_ON || c == C_SRC);
                late += c == C_LATE;
                wait += c == C_WAIT;
                miss += c == C_MISS;
            }
            CellLine a1, a2, a3;
            a1.add(" people " + std::to_string(in.size()));
            a2.add(" on time " + std::to_string(on), C_ON);
            a2.add(" late " + std::to_string(late), C_LATE);
            a3.add(" wait " + std::to_string(wait), C_WAIT);
            a3.add(" miss " + std::to_string(miss), C_MISS);
            cell[a] = {cell[a][0], a1, a2, a3};
        }
    }
    std::string hz = repeat("─", CW);
    auto border = [&](const char* l, const char* m, const char* r) {
        std::string s = std::string(" ") + l;
        for (int c = 0; c < COLS; ++c) s += hz + (c + 1 < COLS ? m : r);
        printf("%s\n", s.c_str());
    };
    border("┌", "┬", "┐");
    for (int row = 0; row < ROWS; ++row) {
        size_t height = 0;
        for (int c = 0; c < COLS; ++c) height = std::max(height, cell[row * COLS + c].size());
        for (size_t i = 0; i < height; ++i) {
            std::string s = " │";
            for (int c = 0; c < COLS; ++c) {
                auto& lines = cell[row * COLS + c];
                if (i < lines.size()) s += lines[i].s + std::string(std::max(0, CW - lines[i].vis), ' ');
                else s += std::string(CW, ' ');
                s += "│";
            }
            printf("%s\n", s.c_str());
        }
        if (row + 1 < ROWS) border("├", "┼", "┤");
    }
    border("└", "┴", "┘");
    printf(" %s server push  %s on time  %s late  %s critical, waiting  %s critical, missed  %s not critical"
           "   (underlined = new this round)\n",
           paint("   ", C_SRC).c_str(), paint("■", C_ON).c_str(), paint("■", C_LATE).c_str(),
           paint("■", C_WAIT).c_str(), paint("■", C_MISS).c_str(), paint("■", C_IDLE).c_str());
}

static std::string describe(const Hospital& h, const Scenario& S, int v, int k) {
    return h.P[v].name + " (" + ROLES[h.P[v].role].name + ", " + ROOM_NAME[S.room[std::min(k, S.H)][v]] +
           ", criticality " + std::to_string(S.crit[v]) + ", deadline " + std::to_string(S.dl[v]) + ")";
}

// ----------------------------------------------------------------------------
// Browser replay (--export): every round is stored as one JSON frame and
// written as viewer/replay_data.js, which viewer/replay.html animates.
// ----------------------------------------------------------------------------
static std::vector<std::string> g_frames;

static std::string jsonStr(const std::string& s) {
    std::string r = "\"";
    for (char ch : s) {
        if (ch == '"' || ch == '\\') { r += '\\'; r += ch; }
        else if ((unsigned char)ch < 0x20) r += ' ';
        else r += ch;
    }
    return r + "\"";
}
static char statusChar(const char* c) {
    return c == C_SRC ? 'S' : c == C_ON ? 'O' : c == C_LATE ? 'L' : c == C_WAIT ? 'W' : c == C_MISS ? 'M' : 'I';
}

static void recordFrame(const Scenario& S, const Planner& P, int k, const std::vector<uint8_t>& arr,
                        const std::vector<char>& isSrc, const std::vector<char>& isNew, const std::vector<int>& parent,
                        int src, const Choice& ch, int missed) {
    if (g_opt.exportPath.empty()) return;
    const int snap = std::min(k, S.H);
    std::ostringstream f;
    f << "{\"r\":" << k << ",\"room\":[";
    for (int v = 0; v < S.N; ++v) f << (v ? "," : "") << S.room[snap][v];
    f << "],\"st\":\"";
    for (int v = 0; v < S.N; ++v) f << statusChar(statusColor(S, arr, isSrc, v, k));
    f << "\",\"new\":[";
    bool first = true;
    for (int v = 0; v < S.N; ++v)
        if (isNew[v]) { f << (first ? "" : ",") << v; first = false; }
    f << "],\"relay\":[";
    first = true;
    for (int v = 0; v < S.N; ++v)
        if (isNew[v] && parent[v] >= 0) { f << (first ? "" : ",") << "[" << parent[v] << "," << v << "]"; first = false; }
    f << "],\"src\":" << src;
    if (src >= 0) {
        f << ",\"score\":" << ch.score << ",\"reach\":" << ch.st.reach << ",\"nw\":" << ch.st.nw << ",\"top\":[";
        for (size_t i = 0; i < ch.top.size(); ++i)
            f << (i ? "," : "") << "[" << ch.top[i].second << "," << ch.top[i].first << "]";
        f << "],\"cands\":" << P.cands << ",\"gpu\":" << (P.useGpu ? 1 : 0) << ",\"kernelMs\":" << P.kernelMs
          << ",\"gpuMs\":" << P.gpuMs << ",\"cpuMs\":" << P.cpuMs << ",\"verify\":" << (P.verify ? 1 : 0)
          << ",\"match\":" << (P.match ? 1 : 0);
    } else if (k > 0) f << ",\"why\":" << jsonStr(ch.why);
    Metrics m = evaluate(S, arr);
    f << ",\"critOn\":" << m.critOn << ",\"informed\":" << m.informed << ",\"missed\":" << missed << "}";
    g_frames.push_back(f.str());
}

typedef std::function<void(int v, int k, int from, bool direct)> Notify;
typedef std::function<std::string()> HeaderExtra;

static void printHeader(const Hospital& h, const Scenario& S, const Opt& o, const Planner& P, int k,
                        const HeaderExtra& extra) {
    if (o.clear) printf("\033[2J\033[H");
    std::string title = " CDA-GB FIRE ALERT SERVER  ·  graph burning with criticality + deadlines ";
    printf("%s\n", paint(title, C_FIRE).c_str());
    std::string when = k == 0 ? "fire just reported" : "round " + std::to_string(k) + "/" + std::to_string(S.H) +
                                                          "  " + fmtTime(k * o.roundSec);
    printf(" Fire in %s  |  %s  |  %d people, %d critical  |  server push budget K=%d\n",
           paint(ROOM_NAME[S.fire], C_NEAR).c_str(), paint(when, BOLD).c_str(), S.N, S.critTotal, o.K);
    printf(" Planner: %s", paint(P.backend, C_CYAN).c_str());
    if (o.fail > 0) printf("  |  unknown link failure %.0f%%", o.fail * 100);
    if (extra) printf("  |  %s", extra().c_str());
    printf("\n");
}

// ----------------------------------------------------------------------------
// One fire event, round by round
// ----------------------------------------------------------------------------
struct EventResult {
    std::vector<uint8_t> arr;
    std::vector<int> parent, seq;
};

static EventResult runEvent(const Hospital& h, const Scenario& S, const Opt& o, Planner& P, const Notify& notify,
                            const HeaderExtra& extra) {
    const int N = S.N;
    EventResult R;
    R.arr.assign(N, INF8);
    R.parent.assign(N, -2);
    std::vector<char> isSrc(N, 0), isNew(N, 0);
    g_frames.clear();
    recordFrame(S, P, 0, R.arr, isSrc, isNew, R.parent, -1, Choice(), 0);

    printHeader(h, S, o, P, 0, extra);
    printMap(h, S, 0, R.arr, isSrc, isNew);
    int nearCount = 0;
    for (int v = 0; v < N; ++v) nearCount += roomDist(S.room[0][v], S.fire) <= 1;
    printf("\n %s Sensor reported fire in %s. %d people are in or next to it; their criticality is raised.\n",
           paint("▸", C_CYAN).c_str(), ROOM_NAME[S.fire], nearCount);
    printf("   Deadline = 7 - criticality. The server may push to one phone per round; phones relay to phones.\n");
    fflush(stdout);
    sleepMs(o.delayMs);

    for (int k = 1; k <= S.H; ++k) {
        Choice ch;
        int src = -1;
        if (k <= o.K) src = P.choose(k, R.arr, ch);
        else ch.why = "push budget used (K=" + std::to_string(o.K) + ")";
        std::vector<uint8_t> before = R.arr;
        realRound(S.exec, k, src, R.arr, R.parent);
        if (src >= 0) { isSrc[src] = 1; R.seq.push_back(src); }
        std::vector<int> newly;
        for (int v = 0; v < N; ++v) {
            isNew[v] = before[v] == INF8 && R.arr[v] != INF8;
            if (isNew[v]) newly.push_back(v);
        }
        for (int v : newly) notify(v, k, R.parent[v], v == src);

        printHeader(h, S, o, P, k, extra);
        printMap(h, S, k, R.arr, isSrc, isNew);
        printf("\n");
        if (src >= 0) {
            char buf[256];
            if (P.useGpu)
                snprintf(buf, sizeof buf, "GPU scored %d candidate phones: kernel %.3f ms, total %.3f ms", P.cands,
                         P.kernelMs, P.gpuMs);
            else snprintf(buf, sizeof buf, "CPU scored %d candidate phones in %.3f ms", P.cands, P.cpuMs);
            std::string line = buf;
            if (P.useGpu && P.verify) {
                snprintf(buf, sizeof buf, " | CPU check %.3f ms, ", P.cpuMs);
                line += buf + (P.match ? paint("identical ✓", C_ON) : paint("MISMATCH", C_MISS));
            }
            printf(" %s Planner: %s\n", paint("▸", C_CYAN).c_str(), line.c_str());
            printf(" %s SERVER ══push══▶ %s\n", paint("▸", C_CYAN).c_str(),
                   paint(describe(h, S, src, k), C_SRC).c_str());
            printf("   score %.2f  (reaches %d phones, %d of them not covered yet)%s\n", ch.score, ch.st.reach,
                   ch.st.nw, ch.score > 0 ? "" : "  - no deadline can still be met, picked for coverage");
            if (!ch.top.empty()) {
                std::string t;
                for (auto& p : ch.top) {
                    snprintf(buf, sizeof buf, "%s %.2f   ", h.P[p.second].name.c_str(), p.first);
                    t += buf;
                }
                printf("   next best: %s\n", t.c_str());
            }
        } else {
            printf(" %s No direct push this round: %s. The alert keeps spreading phone to phone.\n",
                   paint("▸", C_CYAN).c_str(), ch.why.c_str());
        }
        std::string rel;
        int relN = 0;
        for (int v : newly)
            if (R.parent[v] >= 0) {
                if (relN && relN % 8 == 0) rel += "\n          ";
                rel += h.P[R.parent[v]].name + "→" + h.P[v].name + "  ";
                ++relN;
            }
        printf(" %s Relay:  %s\n", paint("▸", C_CYAN).c_str(), relN ? rel.c_str() : "(no new contacts this round)");
        Metrics m = evaluate(S, R.arr);
        int missed = 0;
        for (int v = 0; v < N; ++v)
            if (S.critical[v] && !(R.arr[v] != INF8 && R.arr[v] <= S.dl[v]) && (R.arr[v] != INF8 || k >= S.dl[v]))
                ++missed;  // reached late, or deadline already passed
        printf(" %s Progress: critical on time %s  |  informed %s  |  critical missed %d\n",
               paint("▸", C_CYAN).c_str(), paint(pct(m.critOn, m.critTotal), C_ON).c_str(),
               pct(m.informed, N).c_str(), missed);
        fflush(stdout);
        recordFrame(S, P, k, R.arr, isSrc, isNew, R.parent, src, ch, missed);
        if (m.informed == N) break;
        if (k < S.H) sleepMs(o.delayMs);
    }
    return R;
}

static void printSummary(const Hospital& h, const Scenario& S, const Opt& o, const Planner& P, const EventResult& R) {
    printf("\n%s\n", paint(" RESULT ", C_FIRE).c_str());
    std::string seq;
    for (size_t i = 0; i < R.seq.size(); ++i) seq += (i ? " → " : "") + h.P[R.seq[i]].name;
    printf(" Server push sequence S = [%s]\n", seq.c_str());
    if (P.useGpu) {
        printf(" GPU planner: %d rounds, %.2f ms total", P.calls, P.totGpuMs);
        if (P.verify) printf(" (CPU reference %.2f ms, %d mismatches)", P.totCpuMs, P.mismatches);
        printf("\n");
    } else printf(" CPU planner: %d rounds, %.2f ms total\n", P.calls, P.totCpuMs);

    struct Row { std::string name; Metrics m; };
    std::vector<Row> rows = {{"CDA-GB (this run)", evaluate(S, R.arr)},
                             {"Degree", evaluate(S, runBaseline(S, o.K, "degree", o.seed))},
                             {"Criticality-only", evaluate(S, runBaseline(S, o.K, "crit", o.seed))},
                             {"Random", evaluate(S, runBaseline(S, o.K, "random", o.seed))}};
    printf("\n Same fire, same phones, same K=%d, other ways to pick the server pushes:\n", o.K);
    printf("   %-19s %-20s %-16s %-14s %s\n", "method", "critical on time", "weighted timely", "informed",
           "avg arrival");
    for (auto& r : rows) {
        char a[64], b[64];
        snprintf(a, sizeof a, "%d/%d (%.0f%%)", r.m.critOn, r.m.critTotal, 100 * r.m.tcc);
        snprintf(b, sizeof b, "%d/%d", r.m.informed, r.m.N);
        printf("   %-19s %-20s %-16.3f %-14s round %.2f\n", r.name.c_str(), a, r.m.wtc, b, r.m.avgArrival);
    }
    printf("\n");

    if (o.exportPath.empty()) return;
    std::ostringstream js;
    js << "window.REPLAY = {\"fire\":" << S.fire << ",\"rooms\":[";
    for (int a = 0; a < NROOMS; ++a) js << (a ? "," : "") << jsonStr(ROOM_NAME[a]);
    js << "],\"cols\":" << COLS << ",\"N\":" << S.N << ",\"H\":" << S.H << ",\"K\":" << o.K
       << ",\"roundSec\":" << o.roundSec << ",\"critTotal\":" << S.critTotal << ",\"seed\":" << o.seed
       << ",\"fail\":" << o.fail << ",\"backend\":" << jsonStr(P.backend) << ",\"people\":[";
    for (int v = 0; v < S.N; ++v)
        js << (v ? "," : "") << "[" << jsonStr(h.P[v].name) << "," << jsonStr(ROLES[h.P[v].role].name) << ","
           << S.crit[v] << "," << S.dl[v] << "]";
    js << "],\"frames\":[";
    for (size_t i = 0; i < g_frames.size(); ++i) js << (i ? ",\n" : "\n") << g_frames[i];
    js << "],\"compare\":[";
    for (size_t i = 0; i < rows.size(); ++i)
        js << (i ? "," : "") << "[" << jsonStr(rows[i].name) << "," << rows[i].m.critOn << "," << rows[i].m.critTotal
           << "," << rows[i].m.wtc << "," << rows[i].m.informed << "," << rows[i].m.avgArrival << "]";
    js << "],\"plannerTotal\":{\"calls\":" << P.calls << ",\"gpuMs\":" << P.totGpuMs << ",\"cpuMs\":" << P.totCpuMs
       << ",\"mismatches\":" << P.mismatches << ",\"gpu\":" << (P.useGpu ? 1 : 0) << "}};\n";
    std::ofstream out(o.exportPath, std::ios::binary);
    if (!out) {
        fprintf(stderr, " Cannot write %s\n", o.exportPath.c_str());
        return;
    }
    out << js.str();
    printf(" Replay written to %s  ->  open viewer/replay.html in a browser\n\n", o.exportPath.c_str());
}

// ----------------------------------------------------------------------------
// Networking helpers
// ----------------------------------------------------------------------------
static void netInit() {
#ifdef _WIN32
    WSADATA d;
    WSAStartup(MAKEWORD(2, 2), &d);
#else
    signal(SIGPIPE, SIG_IGN);
#endif
}
static bool sendAll(sock_t s, const std::string& m) {
    size_t sent = 0;
    while (sent < m.size()) {
        int n = (int)send(s, m.data() + sent, (int)(m.size() - sent), MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += (size_t)n;
    }
    return true;
}
static bool recvLine(sock_t s, std::string& buf, std::string& line) {
    for (;;) {
        size_t p = buf.find('\n');
        if (p != std::string::npos) {
            line = buf.substr(0, p);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            buf.erase(0, p + 1);
            return true;
        }
        char tmp[1024];
        int n = (int)recv(s, tmp, sizeof tmp, 0);
        if (n <= 0) return false;
        buf.append(tmp, (size_t)n);
    }
}
static std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream is(s);
    while (std::getline(is, cur, d)) out.push_back(cur);
    return out;
}
static sock_t connectTo(const std::string& host, int port) {
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return BAD_SOCK;
    sock_t s = BAD_SOCK;
    for (addrinfo* a = res; a; a = a->ai_next) {
        s = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (s == BAD_SOCK) continue;
        if (connect(s, a->ai_addr, (int)a->ai_addrlen) == 0) break;
        CLOSESOCK(s);
        s = BAD_SOCK;
    }
    freeaddrinfo(res);
    return s;
}

// ----------------------------------------------------------------------------
// Server mode
// ----------------------------------------------------------------------------
struct Server {
    Opt o;
    Hospital h;
    std::string backend;
    std::mutex mu;
    std::condition_variable cv;
    std::deque<int> fires;
    bool busy = false;
    std::map<int, std::vector<sock_t>> phones;
    std::mutex outMu;

    std::string online() {
        std::lock_guard<std::mutex> lk(mu);
        std::string s;
        for (auto& p : phones)
            if (!p.second.empty()) s += (s.empty() ? "" : " ") + h.P[p.first].name;
        return s.empty() ? "no phones online" : "phones online: " + s;
    }
    void log(const std::string& msg) {
        std::lock_guard<std::mutex> lk(mu);
        if (busy) return;  // keep the animation clean while an event runs
        std::lock_guard<std::mutex> lo(outMu);
        printf(" %s %s\n", paint("[net]", C_CYAN).c_str(), msg.c_str());
        fflush(stdout);
    }
    void sendTo(int v, const std::string& msg) {
        std::lock_guard<std::mutex> lk(mu);
        auto it = phones.find(v);
        if (it == phones.end()) return;
        auto& vec = it->second;
        vec.erase(std::remove_if(vec.begin(), vec.end(), [&](sock_t s) { return !sendAll(s, msg); }), vec.end());
    }
    void reportFire(int room) {
        std::lock_guard<std::mutex> lk(mu);
        fires.push_back(room);
        cv.notify_all();
    }

    void client(sock_t s) {
        std::string buf, line;
        if (!recvLine(s, buf, line)) { CLOSESOCK(s); return; }
        auto sp = line.find(' ');
        std::string cmd = lower(line.substr(0, sp)), arg = sp == std::string::npos ? "" : line.substr(sp + 1);
        if (cmd == "hello") {
            int v = findPerson(h, arg);
            if (v < 0) {
                sendAll(s, "ERR|unknown device '" + arg + "'\n");
                CLOSESOCK(s);
                return;
            }
            {
                std::lock_guard<std::mutex> lk(mu);
                phones[v].push_back(s);
            }
            sendAll(s, "WELCOME|" + std::to_string(v) + "|" + h.P[v].name + "|" + ROLES[h.P[v].role].name + "|" +
                           ROOM_NAME[h.room0[v]] + "\n");
            log("phone " + h.P[v].name + " connected (" + ROLES[h.P[v].role].name + ", " + ROOM_NAME[h.room0[v]] + ")");
            while (recvLine(s, buf, line)) {}  // keep the connection until the phone leaves
            {
                std::lock_guard<std::mutex> lk(mu);
                auto& vec = phones[v];
                vec.erase(std::remove(vec.begin(), vec.end(), s), vec.end());
            }
            log("phone " + h.P[v].name + " disconnected");
        } else if (cmd == "fire") {
            int room = findRoom(arg);
            if (room < 0) sendAll(s, "ERR|unknown room '" + arg + "'\n");
            else {
                sendAll(s, std::string("OK|fire in ") + ROOM_NAME[room] + " reported to the alert server\n");
                log(std::string("sensor reported FIRE in ") + ROOM_NAME[room]);
                reportFire(room);
            }
        } else sendAll(s, "ERR|expected 'HELLO <phone>' or 'FIRE <room>'\n");
        CLOSESOCK(s);
    }

    void printIdle() {
        std::lock_guard<std::mutex> lo(outMu);
        printf("\n%s\n", paint(" CDA-GB FIRE ALERT SERVER ", C_FIRE).c_str());
        printf(" Listening on port %d  |  planner: %s  |  %d people, seed %llu\n", o.port,
               paint(backend, C_CYAN).c_str(), h.N, (unsigned long long)o.seed);
        printf(" Phones (name = what you pass to --id):\n");
        for (int r = 0; r < NROLES; ++r) {
            std::string line;
            for (int v = 0; v < h.N; ++v)
                if (h.P[v].role == r) line += h.P[v].name + "@" + ROOM_NAME[h.room0[v]] + "  ";
            printf("   %-12s %s\n", ROLES[r].name, line.c_str());
        }
        printf(" Rooms: ");
        for (int a = 0; a < NROOMS; ++a) printf("%s%s", ROOM_NAME[a], a + 1 < NROOMS ? ", " : "\n");
        printf(" Waiting for a fire report.  Sensor:  firealert sensor --room ICU\n");
        printf("                             Or type: fire <room>  |  list  |  quit\n\n");
        fflush(stdout);
    }
};

static int runServer(const Opt& o) {
    netInit();
    Server srv;
    srv.o = o;
    srv.h = buildHospital(o.people, o.seed);
    {
        Scenario probe = makeScenario(srv.h, 0, 1, 0, o.seed);
        Planner p;
        p.init(probe, o.w, !o.cpu, false);
        srv.backend = p.backend;
        p.release();
    }
    sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof yes);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((unsigned short)o.port);
    if (bind(ls, (sockaddr*)&addr, sizeof addr) != 0 || listen(ls, 64) != 0) {
        fprintf(stderr, "Cannot listen on port %d (already in use?)\n", o.port);
        return 1;
    }
    std::thread([&srv, ls]() {
        for (;;) {
            sock_t c = accept(ls, nullptr, nullptr);
            if (c == BAD_SOCK) continue;
            std::thread(&Server::client, &srv, c).detach();
        }
    }).detach();
    std::thread([&srv]() {
        std::string line;
        while (std::getline(std::cin, line)) {
            auto parts = split(line, ' ');
            if (parts.empty()) continue;
            std::string c = lower(parts[0]);
            if (c == "quit" || c == "exit") std::exit(0);
            else if (c == "list") srv.printIdle();
            else if (c == "fire" && parts.size() > 1) {
                int room = findRoom(parts[1]);
                if (room < 0) printf(" unknown room '%s'\n", parts[1].c_str());
                else srv.reportFire(room);
            } else printf(" commands: fire <room> | list | quit\n");
        }
    }).detach();

    srv.printIdle();
    for (int event = 1;; ++event) {
        int fire;
        {
            std::unique_lock<std::mutex> lk(srv.mu);
            srv.cv.wait(lk, [&] { return !srv.fires.empty(); });
            fire = srv.fires.front();
            srv.fires.clear();  // one fire at a time; repeated reports are merged
            srv.busy = true;
        }
        Scenario S = makeScenario(srv.h, fire, o.H, o.fail, o.seed + 7919ULL * event);
        Planner P;
        P.init(S, o.w, !o.cpu, o.verify);
        Notify notify = [&](int v, int k, int from, bool direct) {
            int d = roomDist(S.room[std::min(k, S.H)][v], S.fire);
            std::string zone = d == 0 ? "You are IN the fire zone - leave now. " : d == 1 ? "The fire is next to you. " : "";
            std::ostringstream m;
            m << "ALERT|" << k << "|" << (direct ? "DIRECT" : "RELAY") << "|" << (direct ? "SERVER" : srv.h.P[from].name)
              << "|" << ROOM_NAME[S.fire] << "|" << S.dl[v] << "|" << (k <= S.dl[v] ? 1 : 0) << "|" << S.crit[v] << "|"
              << ROOM_NAME[S.room[std::min(k, S.H)][v]] << "|" << zone << ROLES[srv.h.P[v].role].action << "|"
              << k * o.roundSec << "\n";
            srv.sendTo(v, m.str());
        };
        EventResult R = runEvent(srv.h, S, o, P, notify, [&] { return srv.online(); });
        printSummary(srv.h, S, o, P, R);
        for (int v = 0; v < S.N; ++v) {
            std::string st = R.arr[v] == INF8 ? "NOT_REACHED" : R.arr[v] <= S.dl[v] ? "ON_TIME" : "LATE";
            srv.sendTo(v, "END|" + st + "|" + ROOM_NAME[S.fire] + "|" + std::to_string(R.arr[v] == INF8 ? 0 : R.arr[v]) +
                              "|" + std::to_string(S.dl[v]) + "\n");
        }
        P.release();
        {
            std::lock_guard<std::mutex> lk(srv.mu);
            srv.busy = false;
        }
        printf(" Event finished. Waiting for the next fire report (fire <room> | list | quit).\n\n");
        fflush(stdout);
    }
}

// ----------------------------------------------------------------------------
// Phone mode: one thread per simulated phone
// ----------------------------------------------------------------------------
static std::mutex g_print;

static std::vector<std::string> wrap(const std::string& s, size_t width) {
    std::vector<std::string> out;
    std::string line;
    for (auto& word : split(s, ' ')) {
        if (!line.empty() && line.size() + 1 + word.size() > width) { out.push_back(line); line.clear(); }
        line += (line.empty() ? "" : " ") + word;
    }
    if (!line.empty()) out.push_back(line);
    return out;
}
static void box(const std::vector<std::string>& lines, const char* c) {
    const size_t W = 62;
    std::string out = paint(" ╔" + repeat("═", (int)W + 2) + "╗", c) + "\n";
    for (auto& l : lines)
        out += paint(" ║ ", c) + l + std::string(W > l.size() ? W - l.size() : 0, ' ') + paint(" ║", c) + "\n";
    out += paint(" ╚" + repeat("═", (int)W + 2) + "╝", c) + "\n";
    printf("%s", out.c_str());
}

static void phoneThread(std::string id, std::string host, int port) {
    sock_t s = connectTo(host, port);
    if (s == BAD_SOCK) {
        std::lock_guard<std::mutex> lk(g_print);
        printf(" [%s] cannot reach the alert server at %s:%d (is it running?)\n", id.c_str(), host.c_str(), port);
        return;
    }
    sendAll(s, "HELLO " + id + "\n");
    std::string buf, line, me = id;
    while (recvLine(s, buf, line)) {
        auto f = split(line, '|');
        std::lock_guard<std::mutex> lk(g_print);
        if (f[0] == "WELCOME" && f.size() >= 5) {
            me = f[2];
            printf(" 📱 %s (%s), currently in %s - connected, waiting for alerts...\n", paint(me, BOLD).c_str(),
                   f[3].c_str(), f[4].c_str());
        } else if (f[0] == "ALERT" && f.size() >= 11) {
            bool onTime = f[6] == "1";
            std::vector<std::string> L = {
                "FIRE ALERT  -  fire in " + f[4],
                "To: " + me + ", now in " + f[8] + "  (criticality " + f[7] + ")",
                "Received in round " + f[1] + " (" + fmtTime(atoi(f[10].c_str())) + ") " +
                    (f[2] == "DIRECT" ? "directly from the SERVER" : "relayed by phone " + f[3]),
                "Your deadline: round " + f[5] + "  ->  " + (onTime ? "ON TIME" : "LATE"),
                ""};
            for (auto& w : wrap("Action: " + f[9], 62)) L.push_back(w);
            printf("\a");
            box(L, onTime ? C_MISS : C_LATE);
        } else if (f[0] == "END" && f.size() >= 5) {
            std::string st = f[1] == "ON_TIME" ? paint("reached on time", C_ON)
                             : f[1] == "LATE"  ? paint("reached late", C_LATE)
                                               : paint("NOT reached", C_MISS);
            printf(" 📱 %s  event over (fire in %s): %s", me.c_str(), f[2].c_str(), st.c_str());
            if (f[1] != "NOT_REACHED") printf(" - round %s, deadline %s", f[3].c_str(), f[4].c_str());
            printf("\n\n");
        } else if (f[0] == "ERR") {
            printf(" [%s] server: %s\n", id.c_str(), f.size() > 1 ? f[1].c_str() : "error");
        }
        fflush(stdout);
    }
    std::lock_guard<std::mutex> lk(g_print);
    printf(" 📱 %s  connection to the server closed\n", me.c_str());
    CLOSESOCK(s);
}

static int runPhone(const Opt& o) {
    netInit();
    if (o.id.empty()) {
        fprintf(stderr, "phone needs --id <name>[,<name>...], e.g. --id D01 or --id D01,N03,P05\n");
        return 1;
    }
    std::vector<std::thread> th;
    for (auto& id : split(o.id, ','))
        if (!id.empty()) th.emplace_back(phoneThread, id, o.host, o.port);
    for (auto& t : th) t.join();
    return 0;
}

static int runSensor(const Opt& o) {
    netInit();
    std::string room = o.room.empty() ? o.fire : o.room;
    sock_t s = connectTo(o.host, o.port);
    if (s == BAD_SOCK) {
        fprintf(stderr, "Cannot reach the alert server at %s:%d\n", o.host.c_str(), o.port);
        return 1;
    }
    sendAll(s, "FIRE " + room + "\n");
    std::string buf, line;
    if (recvLine(s, buf, line)) {
        auto f = split(line, '|');
        printf(" 🔥 sensor: %s\n", f.size() > 1 ? f[1].c_str() : line.c_str());
    }
    CLOSESOCK(s);
    return 0;
}

static int runDemo(const Opt& o) {
    int fire = findRoom(o.fire);
    if (fire < 0) {
        fprintf(stderr, "Unknown room '%s'\n", o.fire.c_str());
        return 1;
    }
    Hospital h = buildHospital(o.people, o.seed);
    Scenario S = makeScenario(h, fire, o.H, o.fail, o.seed + 7919ULL);
    Planner P;
    P.init(S, o.w, !o.cpu, o.verify);
    EventResult R = runEvent(h, S, o, P, [](int, int, int, bool) {}, nullptr);
    printSummary(h, S, o, P, R);
    P.release();
    return 0;
}

// ----------------------------------------------------------------------------
static void usage() {
    printf(
        "Hospital fire-alert server with CDA-GB graph burning (CUDA, simulation)\n\n"
        "  firealert demo   [--fire ICU]            everything in one terminal\n"
        "  firealert server [--port 9000]           alert server (waits for a fire report)\n"
        "  firealert phone  --id D01[,N03,...]      simulated phone(s)\n"
        "  firealert sensor --room ICU              fire sensor -> server\n\n"
        "Options:\n"
        "  --people 40    people/phones in the hospital (>= 12)\n"
        "  --seed 7       random seed (same seed = same hospital on every machine)\n"
        "  --K 4          server direct pushes (one per round)\n"
        "  --H 10         rounds simulated (6..24)\n"
        "  --round-sec 30 seconds per round (display only)\n"
        "  --w 0.3        overlap weight\n"
        "  --fail 0       fraction of contacts that silently fail (unknown to the server)\n"
        "  --delay 1500   ms pause between rounds   --no-clear  print rounds one after another\n"
        "  --cpu          force the CPU planner     --no-verify skip the CPU check of GPU results\n"
        "  --no-color     plain text output\n"
        "  --export viewer/replay_data.js           save the event for the browser replay (viewer/replay.html)\n"
        "  --host 127.0.0.1 --port 9000             server address for phone / sensor\n"
        "Rooms: ICU ER OT Radiology Ward-A NurseStn Ward-B Lab Pharmacy Lobby Admin Canteen\n");
}

int main(int argc, char** argv) {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (GetConsoleMode(hOut, &mode)) SetConsoleMode(hOut, mode | 0x0004);  // ANSI colours
    SetConsoleOutputCP(65001);                                            // UTF-8
#endif
    Opt& o = g_opt;
    if (argc >= 2) o.mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", a.c_str()); exit(1); }
            return argv[++i];
        };
        if (a == "--people") o.people = atoi(next().c_str());
        else if (a == "--seed") o.seed = strtoull(next().c_str(), nullptr, 10);
        else if (a == "--K") o.K = atoi(next().c_str());
        else if (a == "--H") o.H = atoi(next().c_str());
        else if (a == "--w") o.w = atof(next().c_str());
        else if (a == "--fail") o.fail = atof(next().c_str());
        else if (a == "--delay") o.delayMs = atoi(next().c_str());
        else if (a == "--round-sec") o.roundSec = atoi(next().c_str());
        else if (a == "--port") o.port = atoi(next().c_str());
        else if (a == "--host") o.host = next();
        else if (a == "--fire") o.fire = next();
        else if (a == "--room") o.room = next();
        else if (a == "--id") o.id = next();
        else if (a == "--export") o.exportPath = next();
        else if (a == "--cpu") o.cpu = true;
        else if (a == "--no-verify") o.verify = false;
        else if (a == "--no-clear") o.clear = false;
        else if (a == "--no-color") o.color = false;
        else { fprintf(stderr, "unknown option %s\n\n", a.c_str()); usage(); return 1; }
    }
    o.people = std::max(12, o.people);
    o.H = std::min(24, std::max(6, o.H));
    o.K = std::max(0, o.K);

    if (o.mode == "demo") return runDemo(o);
    if (o.mode == "server") return runServer(o);
    if (o.mode == "phone") return runPhone(o);
    if (o.mode == "sensor") return runSensor(o);
    usage();
    return o.mode == "help" || o.mode == "--help" || o.mode == "-h" ? 0 : 1;
}
