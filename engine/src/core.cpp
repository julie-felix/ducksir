// ducksim 执行核：单文件、arena 分配、零全局态。
//
// 对应设计：docs/plans/2026-11-10-botlab-replica.md 的 D7–D11。
// 语义规格：contract/plan/README.md；numpy 参考实现：sim/plan/oracle.py（对拍 ≤1e-6）。
//
// 结构：blob = 头（48B）+ required 表 + 节点表（32B/条）+ 参数表（float32）。
// 加载时一次性把 arena、参数、history 状态全部分配好；step 里只有 memcpy 和逐元素运算。

#include "ducksim.h"

#include <cmath>
#include <cstring>
#include <new>

namespace {

constexpr uint32_t kMagic = 0x4D495344u;  // 'DSIM'（小端读成 0x4D495344 即 'D','S','I','M'）
constexpr uint16_t kVersion = 1;
constexpr size_t kHeaderSize = 48;
constexpr size_t kNodeRecSize = 32;
constexpr int kMaxInputs = 4;

struct NodeIn {
    uint16_t off;
    uint16_t len;
};

struct Node {
    uint16_t type;
    uint8_t n_in;
    uint8_t flags;
    NodeIn in[kMaxInputs];
    uint16_t out_off;
    uint16_t out_len;
    uint16_t param_off;
    uint16_t param_len;
};

struct Required {
    uint16_t type;
    uint16_t min_version;
};

}  // namespace

struct ds_plan {
    uint16_t node_count = 0;
    uint16_t input_dim = 0;
    uint16_t output_dim = 0;
    uint32_t arena_len = 0;
    uint32_t param_len = 0;

    Node* nodes = nullptr;
    float* params = nullptr;
    float* arena = nullptr;  // 每步清零后重算（确定性）
    int32_t* sink_index = nullptr;

    // history 节点的环形缓冲：每步写一帧，同一时刻读 n 帧
    struct HistState {
        uint16_t node_index;
        uint16_t frames;
        uint16_t frame_len;
        uint16_t head;
        float* buf;  // frames * frame_len
    };
    HistState* hists = nullptr;
    uint32_t hist_count = 0;

    // 状态机：每个节点一份 {state, steps}，在 ds_reset 里回到 initial
    struct SmState {
        uint16_t node_index;
        int32_t state;
        int32_t steps;
        int32_t initial;
    };
    SmState* sms = nullptr;
    uint32_t sm_count = 0;
    uint16_t step_count = 0;
};

namespace {

uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
uint64_t rd64(const uint8_t* p) {
    return static_cast<uint64_t>(rd32(p)) | (static_cast<uint64_t>(rd32(p + 4)) << 32);
}

// 与 Python 侧 fnv1a64 完全一致（非加密用途，只防串版本/改包）
uint64_t fnv1a64(const uint8_t* data, size_t len) {
    uint64_t h = 0xCBF29CE484222325ull;
    for (size_t i = 0; i < len; ++i) {
        h ^= data[i];
        h *= 0x100000001B3ull;
    }
    return h;
}

bool known_type(uint16_t t) { return t >= DS_NODE_OBS && t <= DS_NODE_MOTION; }

void set_err(char* err, size_t err_len, const char* msg) {
    if (!err || err_len == 0) return;
    std::strncpy(err, msg, err_len - 1);
    err[err_len - 1] = '\0';
}

}  // namespace

uint16_t ds_abi_version(void) { return DUCKSIM_ABI; }

ds_plan* ds_load(const void* blob, size_t len, char* err, size_t err_len) {
    if (!blob || len < kHeaderSize) {
        set_err(err, err_len, "blob 太短");
        return nullptr;
    }
    const uint8_t* b = static_cast<const uint8_t*>(blob);
    if (rd32(b) != kMagic) {
        set_err(err, err_len, "magic 不是 DSIM");
        return nullptr;
    }
    const uint16_t version = rd16(b + 4);
    const uint16_t abi = rd16(b + 6);
    const uint16_t node_count = rd16(b + 8);
    const uint16_t req_count = rd16(b + 10);
    const uint16_t input_dim = rd16(b + 12);
    const uint16_t output_dim = rd16(b + 14);
    const uint32_t arena_len = rd32(b + 16);
    const uint32_t param_len = rd32(b + 20);
    const uint64_t digest = rd64(b + 40);

    if (version != kVersion) {
        set_err(err, err_len, "plan version 不支持");
        return nullptr;
    }
    if (abi != DUCKSIM_ABI) {
        set_err(err, err_len, "engine ABI 不匹配");
        return nullptr;
    }
    if (node_count == 0 || input_dim == 0 || output_dim == 0) {
        set_err(err, err_len, "空图");
        return nullptr;
    }

    const size_t body_off = kHeaderSize;
    const size_t need = body_off + static_cast<size_t>(req_count) * 4 +
                        static_cast<size_t>(node_count) * kNodeRecSize +
                        static_cast<size_t>(param_len) * 4;
    if (len < need) {
        set_err(err, err_len, "长度不足");
        return nullptr;
    }
    if (fnv1a64(b + body_off, len - body_off) != digest) {
        set_err(err, err_len, "digest 不匹配");
        return nullptr;
    }

    // required 表：核必须认这些节点，且自己的版本不低于要求
    size_t pos = body_off;
    for (uint16_t i = 0; i < req_count; ++i, pos += 4) {
        const uint16_t t = rd16(b + pos);
        const uint16_t need_v = rd16(b + pos + 2);
        if (!known_type(t)) {
            set_err(err, err_len, "required 里有未知节点类型");
            return nullptr;
        }
        if (need_v > kVersion) {
            set_err(err, err_len, "required 节点版本高于本核");
            return nullptr;
        }
    }

    ds_plan* p = new (std::nothrow) ds_plan();
    if (!p) {
        set_err(err, err_len, "分配 plan 失败");
        return nullptr;
    }
    p->node_count = node_count;
    p->input_dim = input_dim;
    p->output_dim = output_dim;
    p->arena_len = arena_len;
    p->param_len = param_len;
    p->nodes = new (std::nothrow) Node[node_count]();
    p->params = new (std::nothrow) float[param_len ? param_len : 1]();
    p->arena = new (std::nothrow) float[arena_len ? arena_len : 1]();
    p->sink_index = new (std::nothrow) int32_t[1]();
    if (!p->nodes || !p->params || !p->arena || !p->sink_index) {
        ds_free(p);
        set_err(err, err_len, "分配节点失败");
        return nullptr;
    }
    p->sink_index[0] = -1;

    // 节点表
    for (uint16_t i = 0; i < node_count; ++i, pos += kNodeRecSize) {
        const uint8_t* r = b + pos;
        Node& n = p->nodes[i];
        n.type = rd16(r);
        n.n_in = r[2];
        n.flags = r[3];
        for (int k = 0; k < kMaxInputs; ++k) {
            n.in[k].off = rd16(r + 4 + k * 4);
            n.in[k].len = rd16(r + 6 + k * 4);
        }
        n.out_off = rd16(r + 20);
        n.out_len = rd16(r + 22);
        n.param_off = rd16(r + 24);
        n.param_len = rd16(r + 26);

        if (!known_type(n.type)) {
            ds_free(p);
            set_err(err, err_len, "节点类型未知");
            return nullptr;
        }
        if (n.n_in > kMaxInputs) {
            ds_free(p);
            set_err(err, err_len, "输入数超过 4");
            return nullptr;
        }
        if (n.out_len == 0 || static_cast<uint32_t>(n.out_off) + n.out_len > arena_len) {
            ds_free(p);
            set_err(err, err_len, "节点输出越界");
            return nullptr;
        }
        if (p->param_len &&
            (static_cast<uint32_t>(n.param_off) + n.param_len > p->param_len)) {
            ds_free(p);
            set_err(err, err_len, "节点参数越界");
            return nullptr;
        }
        // 输入必须来自前序节点输出（保证声明顺序即拓扑序）
        for (int k = 0; k < n.n_in; ++k) {
            bool ok = false;
            for (uint16_t j = 0; j < i && !ok; ++j) {
                const Node& prev = p->nodes[j];
                if (prev.out_off == n.in[k].off && prev.out_len == n.in[k].len) ok = true;
            }
            if (!ok) {
                ds_free(p);
                set_err(err, err_len, "输入不是前序节点的输出（图不是拓扑序）");
                return nullptr;
            }
        }
        if (n.type == DS_NODE_OUT_ACTION) {
            if (p->sink_index[0] >= 0) {
                ds_free(p);
                set_err(err, err_len, "out_action 多于一个");
                return nullptr;
            }
            if (n.out_len != output_dim) {
                ds_free(p);
                set_err(err, err_len, "out_action 长度与 output_dim 不符");
                return nullptr;
            }
            p->sink_index[0] = i;
        }
        if (n.type == DS_NODE_MOTION) {
            // 参数表：[n_frames, dim, fps, loop] + n_frames*dim 个 float。
            // 这里只查最小长度（内容在 ds_step 里 O(1) 校验，避免在加载期算乘积）。
            if (n.param_len < 4) {
                ds_free(p);
                set_err(err, err_len, "motion 参数表至少要 4 个 float");
                return nullptr;
            }
            if (n.n_in != 1) {
                ds_free(p);
                set_err(err, err_len, "motion 只吃 1 个输入（时间）");
                return nullptr;
            }
        }
        if (n.type == DS_NODE_STATE_MACHINE && n.out_len != 2) {
            ds_free(p);
            set_err(err, err_len, "state_machine 输出必须是 2 维 [state, steps]");
            return nullptr;
        }
        if (n.type == DS_NODE_MODEL && !(n.flags & DS_FLAG_PASS_THROUGH) && n.param_len < 1) {
            ds_free(p);
            set_err(err, err_len, "model 节点缺 model_id");
            return nullptr;
        }
    }
    if (p->sink_index[0] < 0) {
        ds_free(p);
        set_err(err, err_len, "没有 out_action");
        return nullptr;
    }
    // 参数表
    if (param_len) {
        const uint8_t* pp = b + pos;
        for (uint32_t i = 0; i < param_len; ++i) {
            uint32_t bits = rd32(pp + i * 4);
            std::memcpy(&p->params[i], &bits, 4);
        }
    }

    // history 状态
    uint32_t hist = 0;
    for (uint16_t i = 0; i < node_count; ++i) {
        if (p->nodes[i].type == DS_NODE_HISTORY) ++hist;
    }
    p->hist_count = hist;
    if (hist) {
        p->hists = new (std::nothrow) ds_plan::HistState[hist]();
        if (!p->hists) {
            ds_free(p);
            set_err(err, err_len, "分配 history 状态失败");
            return nullptr;
        }
        uint32_t k = 0;
        for (uint16_t i = 0; i < node_count; ++i) {
            const Node& n = p->nodes[i];
            if (n.type != DS_NODE_HISTORY) continue;
            if (n.param_len < 1) {
                ds_free(p);
                set_err(err, err_len, "history 缺 frames");
                return nullptr;
            }
            const int frames = static_cast<int>(p->params[n.param_off]);
            if (frames < 1 || frames > 1024 || (n.out_len % frames) != 0) {
                ds_free(p);
                set_err(err, err_len, "history frames 非法");
                return nullptr;
            }
            p->hists[k].node_index = i;
            p->hists[k].frames = static_cast<uint16_t>(frames);
            p->hists[k].frame_len = static_cast<uint16_t>(n.out_len / frames);
            p->hists[k].head = 0;
            p->hists[k].buf = new (std::nothrow) float[static_cast<size_t>(frames) * p->hists[k].frame_len]();
            if (!p->hists[k].buf) {
                ds_free(p);
                set_err(err, err_len, "分配 history 缓冲失败");
                return nullptr;
            }
            ++k;
        }
    }
    // 状态机状态
    uint32_t ns = 0;
    for (uint16_t i = 0; i < node_count; ++i) {
        if (p->nodes[i].type == DS_NODE_STATE_MACHINE) ++ns;
    }
    p->sm_count = ns;
    if (ns) {
        p->sms = new (std::nothrow) ds_plan::SmState[ns]();
        if (!p->sms) {
            ds_free(p);
            set_err(err, err_len, "分配状态机失败");
            return nullptr;
        }
        uint32_t k = 0;
        for (uint16_t i = 0; i < node_count; ++i) {
            const Node& n = p->nodes[i];
            if (n.type != DS_NODE_STATE_MACHINE) continue;
            if (n.param_len < 3) {
                ds_free(p);
                set_err(err, err_len, "state_machine 参数不足");
                return nullptr;
            }
            const int initial = static_cast<int>(p->params[n.param_off + 1]);
            p->sms[k].node_index = i;
            p->sms[k].initial = initial;
            p->sms[k].state = initial;
            p->sms[k].steps = 0;
            ++k;
        }
    }

    return p;
}

void ds_free(ds_plan* p) {
    if (!p) return;
    if (p->hists) {
        for (uint32_t i = 0; i < p->hist_count; ++i) delete[] p->hists[i].buf;
        delete[] p->hists;
    }
    delete[] p->sms;
    delete[] p->nodes;
    delete[] p->params;
    delete[] p->arena;
    delete[] p->sink_index;
    delete p;
}

int ds_info_of(const ds_plan* p, ds_info* out) {
    if (!p || !out) return DS_ERR_NULL;
    out->version = kVersion;
    out->abi = DUCKSIM_ABI;
    out->node_count = p->node_count;
    out->input_dim = p->input_dim;
    out->output_dim = p->output_dim;
    out->arena_len = p->arena_len;
    out->param_len = p->param_len;
    return DS_OK;
}

int ds_reset(ds_plan* p) {
    if (!p) return DS_ERR_NULL;
    for (uint32_t i = 0; i < p->hist_count; ++i) {
        const size_t n = static_cast<size_t>(p->hists[i].frames) * p->hists[i].frame_len;
        std::memset(p->hists[i].buf, 0, n * sizeof(float));
        p->hists[i].head = 0;
    }
    for (uint32_t i = 0; i < p->sm_count; ++i) {
        p->sms[i].state = p->sms[i].initial;
        p->sms[i].steps = 0;
    }
    p->step_count = 0;
    return DS_OK;
}

namespace {

// 环形缓冲的读写：写当前帧，再按「最老在前」铺开
void hist_push(ds_plan::HistState& h, const float* x) {
    std::memcpy(h.buf + static_cast<size_t>(h.head) * h.frame_len, x,
                sizeof(float) * h.frame_len);
    h.head = static_cast<uint16_t>((h.head + 1) % h.frames);
}

void hist_read(const ds_plan::HistState& h, float* out) {
    for (uint16_t k = 0; k < h.frames; ++k) {
        const uint16_t idx = static_cast<uint16_t>((h.head + k) % h.frames);
        std::memcpy(out + static_cast<size_t>(k) * h.frame_len,
                    h.buf + static_cast<size_t>(idx) * h.frame_len,
                    sizeof(float) * h.frame_len);
    }
}

}  // namespace

int ds_step(ds_plan* p, const float* obs, float* action, const ds_backend* backend) {
    if (!p || !obs || !action) return DS_ERR_NULL;

    // arena 每步清零：节点输出全量重写，读到未写区域一定是 bug，归零让它可复现
    std::memset(p->arena, 0, sizeof(float) * (p->arena_len ? p->arena_len : 1));

    uint32_t hist_cursor = 0;
    uint32_t sm_cursor = 0;
    for (uint16_t i = 0; i < p->node_count; ++i) {
        const Node& n = p->nodes[i];
        float* out = p->arena + n.out_off;
        const float* in0 = n.n_in > 0 ? p->arena + n.in[0].off : nullptr;
        const float* prm = p->params + n.param_off;

        switch (n.type) {
            case DS_NODE_OBS: {
                if (n.param_len < 2) return DS_ERR_BAD_BLOB;
                const int src = static_cast<int>(prm[0]);
                const int need = static_cast<int>(prm[1]);
                if (src < 0 || need < 0 || src + need > static_cast<int>(p->input_dim) ||
                    need != static_cast<int>(n.out_len)) {
                    return DS_ERR_GRAPH;
                }
                std::memcpy(out, obs + src, sizeof(float) * n.out_len);
                break;
            }
            case DS_NODE_CONCAT: {
                uint16_t written = 0;
                for (uint8_t k = 0; k < n.n_in; ++k) {
                    const NodeIn& e = n.in[k];
                    std::memcpy(out + written, p->arena + e.off, sizeof(float) * e.len);
                    written = static_cast<uint16_t>(written + e.len);
                }
                if (written != n.out_len) return DS_ERR_GRAPH;
                break;
            }
            case DS_NODE_AFFINE: {
                if (n.param_len < 2 * n.out_len) return DS_ERR_BAD_BLOB;
                const float* scale = prm;
                const float* offset = prm + n.out_len;
                for (uint16_t j = 0; j < n.out_len; ++j) {
                    out[j] = in0[j] * scale[j] + offset[j];
                }
                break;
            }
            case DS_NODE_CLIP: {
                if (n.param_len < 2 * n.out_len) return DS_ERR_BAD_BLOB;
                const float* lo = prm;
                const float* hi = prm + n.out_len;
                for (uint16_t j = 0; j < n.out_len; ++j) {
                    const float v = in0[j];
                    out[j] = v < lo[j] ? lo[j] : (v > hi[j] ? hi[j] : v);
                }
                break;
            }
            case DS_NODE_REORDER: {
                if (n.param_len < n.out_len) return DS_ERR_BAD_BLOB;
                for (uint16_t j = 0; j < n.out_len; ++j) {
                    const int src = static_cast<int>(prm[j]);
                    if (src < 0 || src >= static_cast<int>(n.in[0].len)) return DS_ERR_GRAPH;
                    out[j] = in0[src];
                }
                break;
            }
            case DS_NODE_HISTORY: {
                if (hist_cursor >= p->hist_count) return DS_ERR_GRAPH;
                ds_plan::HistState& h = p->hists[hist_cursor++];
                hist_push(h, in0);
                hist_read(h, out);
                break;
            }
            case DS_NODE_MODEL: {
                if (n.flags & DS_FLAG_PASS_THROUGH) {
                    if (n.out_len != n.in[0].len) return DS_ERR_GRAPH;
                    std::memcpy(out, in0, sizeof(float) * n.out_len);
                    break;
                }
                if (!backend || !backend->onnx_run) return DS_ERR_NO_BACKEND;
                const int model_id = n.param_len >= 1 ? static_cast<int>(prm[0]) : 0;
                const int rc = backend->onnx_run(backend->ctx, model_id, in0,
                                                 static_cast<int>(n.in[0].len), out,
                                                 static_cast<int>(n.out_len));
                if (rc != 0) return DS_ERR_GRAPH;
                break;
            }
            case DS_NODE_CONSTANT: {
                if (n.param_len < n.out_len) return DS_ERR_BAD_BLOB;
                if (n.n_in != 0) return DS_ERR_GRAPH;
                std::memcpy(out, prm, sizeof(float) * n.out_len);
                break;
            }
            case DS_NODE_SELECT: {
                if (n.n_in < 2 || n.param_len < 2) return DS_ERR_BAD_BLOB;
                const int n_choices = n.n_in - 1;
                const int width = static_cast<int>(prm[n_choices]);
                const int passive = static_cast<int>(prm[n_choices + 1]);
                if (width != static_cast<int>(n.out_len)) return DS_ERR_GRAPH;
                const float mode_val = in0[0];
                const float* picked = nullptr;
                for (int k = 0; k < n_choices; ++k) {
                    if (prm[k] == mode_val) {
                        picked = p->arena + n.in[1 + k].off;
                        break;
                    }
                }
                if (!picked) {
                    if (passive >= 0 && passive < n_choices) {
                        picked = p->arena + n.in[1 + passive].off;
                    } else {
                        std::memset(out, 0, sizeof(float) * n.out_len);
                        break;
                    }
                }
                std::memcpy(out, picked, sizeof(float) * n.out_len);
                break;
            }
            case DS_NODE_WHERE: {
                if (n.n_in != 3) return DS_ERR_GRAPH;
                const float* cond = p->arena + n.in[0].off;
                const float* a = p->arena + n.in[1].off;
                const float* b = p->arena + n.in[2].off;
                const float* src = cond[0] != 0.0f ? a : b;
                std::memcpy(out, src, sizeof(float) * n.out_len);
                break;
            }
            case DS_NODE_PHASE_CLOCK: {
                if (n.param_len < 1 || n.n_in != 1) return DS_ERR_GRAPH;
                const float period = prm[0];
                if (period < 1.0f) return DS_ERR_BAD_BLOB;
                const float steps = in0[0];
                // 与 Python 的 `steps % period` 对齐：结果始终落在 [0,1)，负数步数也不出负相位
                float phase = std::fmod(steps, period);
                if (phase < 0.0f) phase += period;
                out[0] = phase / period;
                break;
            }
            case DS_NODE_MOTION: {
                // 输入是时间（秒）；按 fps 折算成帧号，取相邻两帧线性插值。
                // loop=1 时环形回绕；否则夹在 [0, frames-1]。
                if (n.param_len < 4) return DS_ERR_BAD_BLOB;
                const int frames = static_cast<int>(prm[0]);
                const int dim = static_cast<int>(prm[1]);
                const float fps = prm[2];
                const bool loop = prm[3] != 0.0f;
                if (frames <= 0 || dim <= 0 || !(fps > 0.0f)) return DS_ERR_BAD_BLOB;
                if (static_cast<uint32_t>(4 + frames * dim) > n.param_len) return DS_ERR_BAD_BLOB;
                const float* table = prm + 4;
                float pos = in0[0] * fps;
                if (!(pos >= 0.0f)) pos = 0.0f;   // 也挡住 NaN
                int i0 = static_cast<int>(std::floor(pos));
                float frac = pos - static_cast<float>(i0);
                int i1;
                if (loop) {
                    i0 %= frames;
                    if (i0 < 0) i0 += frames;
                    i1 = (i0 + 1) % frames;
                } else {
                    if (i0 > frames - 1) { i0 = frames - 1; frac = 0.0f; }
                    i1 = i0 + 1 < frames ? i0 + 1 : i0;
                }
                const float* ra = table + static_cast<size_t>(i0) * dim;
                const float* rb = table + static_cast<size_t>(i1) * dim;
                for (int k = 0; k < dim; ++k) {
                    out[k] = ra[k] * (1.0f - frac) + rb[k] * frac;
                }
                break;
            }
            case DS_NODE_COMPARE: {
                if (n.n_in != 2 || n.param_len < 1) return DS_ERR_GRAPH;
                const float a = in0[0];
                const float b = p->arena[n.in[1].off];
                const int op = static_cast<int>(prm[0]);
                bool res = false;
                switch (op) {
                    case 0: res = a > b; break;
                    case 1: res = a >= b; break;
                    case 2: res = a < b; break;
                    case 3: res = a <= b; break;
                    case 4: res = a == b; break;
                    default: return DS_ERR_BAD_BLOB;
                }
                out[0] = res ? 1.0f : 0.0f;
                break;
            }
            case DS_NODE_STATE_MACHINE: {
                if (sm_cursor >= p->sm_count) return DS_ERR_GRAPH;
                ds_plan::SmState& sm = p->sms[sm_cursor++];
                if (n.param_len < 3) return DS_ERR_BAD_BLOB;
                const int n_tr = static_cast<int>(prm[0]);
                const int n_cond = static_cast<int>(prm[2]);
                if (n.param_len < 3 + n_tr * 6) return DS_ERR_BAD_BLOB;
                if (static_cast<int>(n.n_in) != 1 + n_cond) return DS_ERR_GRAPH;
                const float cmd = in0[0];
                bool switched = false;
                for (int want_kind = 0; want_kind < 3 && !switched; ++want_kind) {
                    for (int t = 0; t < n_tr; ++t) {
                        const float* tr = prm + 3 + t * 6;
                        const int frm = static_cast<int>(tr[0]);
                        const int kind = static_cast<int>(tr[2]);
                        const float val = tr[3];
                        const int ci = static_cast<int>(tr[4]);
                        const bool reset = tr[5] != 0.0f;
                        if (kind != want_kind) continue;
                        if (frm >= 0 && frm != sm.state) continue;
                        bool hit = false;
                        if (kind == 0) {
                            hit = cmd != 0.0f && cmd == val;
                        } else if (kind == 1) {
                            hit = sm.steps >= static_cast<int>(val);
                        } else {
                            if (ci < 0 || ci >= n_cond) continue;
                            const float* cond = p->arena + n.in[1 + ci].off;
                            hit = cond[0] != 0.0f;
                        }
                        if (!hit) continue;
                        sm.state = static_cast<int>(tr[1]);
                        sm.steps = reset ? 0 : sm.steps + 1;
                        switched = true;
                        break;
                    }
                }
                if (!switched) sm.steps += 1;
                out[0] = static_cast<float>(sm.state);
                out[1] = static_cast<float>(sm.steps);
                break;
            }
            case DS_NODE_ADD: {
                for (uint16_t j = 0; j < n.out_len; ++j) out[j] = 0.0f;
                for (uint8_t k = 0; k < n.n_in; ++k) {
                    const float* src = p->arena + n.in[k].off;
                    for (uint16_t j = 0; j < n.out_len; ++j) out[j] += src[j];
                }
                break;
            }
            case DS_NODE_OUT_ACTION: {
                std::memcpy(action, in0, sizeof(float) * n.out_len);
                break;
            }
            default:
                return DS_ERR_UNSUPPORTED;
        }
    }
    p->step_count = static_cast<uint16_t>(p->step_count + 1);
    return DS_OK;
}
