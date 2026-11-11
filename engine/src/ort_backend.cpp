// ORT 后端：把 `ds_backend.onnx_run` 接到 onnxruntime 的 C API 上。
//
// 为什么单独一个 .so：执行核（core.cpp）必须保持零依赖——它要能编到 wasm、能上板子。
// 推理这一段在浏览器里是另一条路（ORT Web），所以只有 native 这一份链 ORT（计划 D8）。
//
// 确定性：intra/inter op 线程都锁 1。多线程归约会让同一份输入算出不同末位，
// 而我们的对拍判据是 1e-6、目标是逐位一致。

#include "ducksim.h"

#include <onnxruntime_c_api.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct OrtCtx {
    const OrtApi* api = nullptr;
    OrtEnv* env = nullptr;
    OrtSessionOptions* opts = nullptr;
    OrtSession* session = nullptr;
    OrtMemoryInfo* mem = nullptr;
    // 支持 1..3 个输入 / 1..3 个输出：递归策略（LSTM/GRU）就是 obs + h + c → action + h + c。
    // 核按「单张量」约定把 [obs,h,c] 拼好、把 [action,h,c] 拆开；这里负责还原成模型的真实张量。
    std::vector<std::string> in_names;
    std::vector<std::string> out_names;
    std::vector<int64_t> in_lens;   // 每个张量的元素数
    std::vector<int64_t> out_lens;
    std::vector<std::vector<int64_t>> in_dims;   // 每个张量的真实形状（[1,34] 这种）
    std::vector<std::vector<int64_t>> out_dims;
    int64_t in_len = 0;    // 合计（核送进来的长度）
    int64_t out_len = 0;   // 合计（核期望拿到的长度）
};

struct Backend {
    ds_backend be{};
    OrtCtx ctx{};                 // 单模型时为 0 号；多模型时每个槽一份，共享 env/opts
    std::vector<OrtCtx> slots;
};

void set_err(char* err, size_t err_len, const char* msg) {
    if (!err || err_len == 0) return;
    std::strncpy(err, msg, err_len - 1);
    err[err_len - 1] = '\0';
}

void set_ort_err(char* err, size_t err_len, const OrtApi* api, OrtStatus* st, const char* what) {
    if (!err || err_len == 0) {
        if (st && api) api->ReleaseStatus(st);
        return;
    }
    const char* msg = st && api ? api->GetErrorMessage(st) : "unknown";
    std::snprintf(err, err_len, "%s: %s", what, msg ? msg : "?");
    if (st && api) api->ReleaseStatus(st);
}

// 取单输入/单输出的名字与元素数；只接受 float32、单 batch 或纯一维
bool describe(const OrtApi* api, Backend* b, char* err, size_t err_len, const char* what) {
    OrtCtx& c = b->ctx;
    OrtStatus* st = nullptr;
    size_t n_in = 0, n_out = 0;
    if ((st = api->SessionGetInputCount(c.session, &n_in))) {
        set_ort_err(err, err_len, api, st, "SessionGetInputCount");
        return false;
    }
    if ((st = api->SessionGetOutputCount(c.session, &n_out))) {
        set_ort_err(err, err_len, api, st, "SessionGetOutputCount");
        return false;
    }
    // 最多 3 进 3 出：obs/h/c 是上限，再多就说明这个模型不是我们这条链上的
    if (n_in < 1 || n_in > 3 || n_out < 1 || n_out > 3) {
        std::snprintf(err, err_len, "%s 只支持 1..3 个输入与输出（实际 %zu/%zu）", what, n_in, n_out);
        return false;
    }

    OrtAllocator* alloc = nullptr;
    if ((st = api->GetAllocatorWithDefaultOptions(&alloc))) {
        set_ort_err(err, err_len, api, st, "GetAllocatorWithDefaultOptions");
        return false;
    }

    auto name_of = [&](int is_input, size_t idx, std::string* out_s) -> bool {
        char* name = nullptr;
        OrtStatus* s2 = is_input ? api->SessionGetInputName(c.session, idx, alloc, &name)
                                 : api->SessionGetOutputName(c.session, idx, alloc, &name);
        if (s2) {
            set_ort_err(err, err_len, api, s2, is_input ? "SessionGetInputName" : "SessionGetOutputName");
            return false;
        }
        *out_s = name;
        if ((s2 = api->AllocatorFree(alloc, name))) {
            set_ort_err(err, err_len, api, s2, "AllocatorFree");
            return false;
        }
        return true;
    };

    auto shape_of = [&](int is_input, size_t idx, int64_t* len,
                        std::vector<int64_t>* keep, const std::string& label) -> bool {
        OrtTypeInfo* info = nullptr;
        OrtStatus* s2 = is_input ? api->SessionGetInputTypeInfo(c.session, idx, &info)
                                 : api->SessionGetOutputTypeInfo(c.session, idx, &info);
        if (s2) {
            set_ort_err(err, err_len, api, s2, "SessionGetTypeInfo");
            return false;
        }
        const OrtTensorTypeAndShapeInfo* ts = nullptr;
        if ((s2 = api->CastTypeInfoToTensorInfo(info, &ts))) {
            set_ort_err(err, err_len, api, s2, "CastTypeInfoToTensorInfo");
            api->ReleaseTypeInfo(info);
            return false;
        }
        ONNXTensorElementDataType dt = ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED;
        if ((s2 = api->GetTensorElementType(ts, &dt))) {
            set_ort_err(err, err_len, api, s2, "GetTensorElementType");
            api->ReleaseTypeInfo(info);
            return false;
        }
        size_t rank = 0;
        if ((s2 = api->GetDimensionsCount(ts, &rank))) {
            set_ort_err(err, err_len, api, s2, "GetDimensionsCount");
            api->ReleaseTypeInfo(info);
            return false;
        }
        std::vector<int64_t> dims(rank > 0 ? rank : 1, 0);
        if ((s2 = api->GetDimensions(ts, dims.data(), rank))) {
            set_ort_err(err, err_len, api, s2, "GetDimensions");
            api->ReleaseTypeInfo(info);
            return false;
        }
        int64_t total = 1;
        for (int64_t d : dims) {
            if (d <= 0) {
                std::snprintf(err, err_len, "%s 有动态维度（%s），本核要求静态形状", label.c_str(),
                              label.c_str());
                api->ReleaseTypeInfo(info);
                return false;
            }
            total *= d;
        }
        if (dt != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            std::snprintf(err, err_len, "只支持 float32 张量");
            api->ReleaseTypeInfo(info);
            return false;
        }
        *len = total;
        keep->assign(dims.begin(), dims.end());
        api->ReleaseTypeInfo(info);
        return true;
    };

    c.in_names.clear();
    c.in_lens.clear();
    c.in_dims.clear();
    c.out_names.clear();
    c.out_lens.clear();
    c.out_dims.clear();
    c.in_len = 0;
    c.out_len = 0;
    for (size_t i = 0; i < n_in; ++i) {
        std::string name;
        if (!name_of(1, i, &name)) return false;
        int64_t len = 0;
        std::vector<int64_t> dims;
        const std::string label = std::string(what) + " 输入 " + std::to_string(i) + "（" + name + "）";
        if (!shape_of(1, i, &len, &dims, label)) return false;
        c.in_names.push_back(name);
        c.in_lens.push_back(len);
        c.in_dims.push_back(dims);
        c.in_len += len;
    }
    for (size_t i = 0; i < n_out; ++i) {
        std::string name;
        if (!name_of(0, i, &name)) return false;
        int64_t len = 0;
        std::vector<int64_t> dims;
        const std::string label = std::string(what) + " 输出 " + std::to_string(i) + "（" + name + "）";
        if (!shape_of(0, i, &len, &dims, label)) return false;
        c.out_names.push_back(name);
        c.out_lens.push_back(len);
        c.out_dims.push_back(dims);
        c.out_len += len;
    }
    return true;
}


int ort_run(void* raw_ctx, int model_id, const float* in, int in_len, float* out, int out_len) {
    auto* b = static_cast<Backend*>(raw_ctx);
    if (!b || !in || !out) return 1;
    // 多模型时按 model_id 取槽；单模型时就是 b->ctx。
    // 注意：**不能**把槽拷回 b->ctx —— 那样析构会把同一个 session 释放两次（踩过，free(): invalid pointer）。
    OrtCtx* c = nullptr;
    if (b->slots.empty()) {
        if (model_id != 0) return 6;
        c = &b->ctx;
    } else {
        if (model_id < 0 || model_id >= static_cast<int>(b->slots.size())) return 6;
        c = &b->slots[static_cast<size_t>(model_id)];
    }
    if (in_len != c->in_len || out_len != c->out_len) return 2;

    const OrtApi* api = c->api;
    const size_t n_in = c->in_lens.size();
    const size_t n_out = c->out_lens.size();

    // 按模型声明的每个张量切一片输入（核送进来的是一整条 [obs,h,c]）
    std::vector<OrtValue*> inputs(n_in, nullptr);
    std::vector<const char*> in_names(n_in, nullptr);
    size_t off = 0;
    for (size_t i = 0; i < n_in; ++i) {
        const size_t len = static_cast<size_t>(c->in_lens[i]);
        OrtStatus* st = api->CreateTensorWithDataAsOrtValue(
            c->mem, const_cast<float*>(in) + off, len * sizeof(float),
            c->in_dims[i].data(), c->in_dims[i].size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
            &inputs[i]);
        if (st) {
            api->ReleaseStatus(st);
            for (size_t k = 0; k < i; ++k) api->ReleaseValue(inputs[k]);
            return 3;
        }
        in_names[i] = c->in_names[i].c_str();
        off += len;
    }

    std::vector<const char*> out_names(n_out, nullptr);
    for (size_t i = 0; i < n_out; ++i) out_names[i] = c->out_names[i].c_str();
    std::vector<OrtValue*> outputs(n_out, nullptr);

    OrtStatus* st = api->Run(c->session, nullptr, in_names.data(), inputs.data(), n_in,
                             out_names.data(), n_out, outputs.data());
    for (OrtValue* v : inputs) api->ReleaseValue(v);
    if (st) {
        api->ReleaseStatus(st);
        return 4;
    }
    size_t out_off = 0;
    for (size_t i = 0; i < n_out; ++i) {
        float* data = nullptr;
        st = api->GetTensorMutableData(outputs[i], reinterpret_cast<void**>(&data));
        if (st) {
            api->ReleaseStatus(st);
            for (OrtValue* v : outputs) api->ReleaseValue(v);
            return 5;
        }
        const size_t len = static_cast<size_t>(c->out_lens[i]);
        std::memcpy(out + out_off, data, len * sizeof(float));
        out_off += len;
        api->ReleaseValue(outputs[i]);
    }
    return 0;
}


}  // namespace

namespace {

// 把一份模型装进 ctx（共享 env/opts，独立 session/名字/维度）
bool load_into(const OrtApi* api, Backend* b, OrtCtx* c, const char* path, char* err, size_t err_len) {
    c->api = api;
    c->env = b->ctx.env;
    c->opts = b->ctx.opts;
    OrtStatus* st = api->CreateSession(c->env, path, c->opts, &c->session);
    if (st) {
        set_ort_err(err, err_len, api, st, "CreateSession");
        return false;
    }
    if ((st = api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &c->mem))) {
        set_ort_err(err, err_len, api, st, "CreateCpuMemoryInfo");
        return false;
    }
    // describe() 读的是 Backend 上的 ctx，这里临时把它指到 c
    OrtCtx saved = b->ctx;
    b->ctx = *c;
    const bool ok = describe(api, b, err, err_len, "模型");
    *c = b->ctx;
    b->ctx = saved;
    return ok;
}

}  // namespace

extern "C" {

ds_backend* ds_ort_create_multi(const char* const* paths, int count, char* err, size_t err_len) {
    if (!paths || count < 1) {
        set_err(err, err_len, "paths 为空");
        return nullptr;
    }
    auto* b = new (std::nothrow) Backend();
    if (!b) {
        set_err(err, err_len, "分配 backend 失败");
        return nullptr;
    }
    const OrtApi* api = OrtGetApiBase()->GetApi(ORT_API_VERSION);
    if (!api) {
        delete b;
        set_err(err, err_len, "OrtGetApiBase 失败（ABI 不匹配？）");
        return nullptr;
    }
    b->ctx.api = api;
    OrtStatus* st = nullptr;
    if ((st = api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "ducksim", &b->ctx.env))) {
        set_ort_err(err, err_len, api, st, "CreateEnv");
        delete b;
        return nullptr;
    }
    if ((st = api->CreateSessionOptions(&b->ctx.opts))) {
        set_ort_err(err, err_len, api, st, "CreateSessionOptions");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->SetIntraOpNumThreads(b->ctx.opts, 1))) {
        set_ort_err(err, err_len, api, st, "SetIntraOpNumThreads");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->SetSessionGraphOptimizationLevel(b->ctx.opts, ORT_ENABLE_ALL))) {
        set_ort_err(err, err_len, api, st, "SetSessionGraphOptimizationLevel");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    b->slots.resize(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        if (!load_into(api, b, &b->slots[static_cast<size_t>(i)], paths[i], err, err_len)) {
            ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
            return nullptr;
        }
    }
    b->be.ctx = b;
    b->be.onnx_run = &ort_run;
    return &b->be;
}

ds_backend* ds_ort_create(const char* model_path, char* err, size_t err_len) {
    if (!model_path) {
        set_err(err, err_len, "model_path 为空");
        return nullptr;
    }
    auto* b = new (std::nothrow) Backend();
    if (!b) {
        set_err(err, err_len, "分配 backend 失败");
        return nullptr;
    }
    const OrtApi* api = OrtGetApiBase()->GetApi(ORT_API_VERSION);
    if (!api) {
        delete b;
        set_err(err, err_len, "OrtGetApiBase 失败（ABI 不匹配？）");
        return nullptr;
    }
    b->ctx.api = api;
    OrtStatus* st = nullptr;
    if ((st = api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "ducksim", &b->ctx.env))) {
        set_ort_err(err, err_len, api, st, "CreateEnv");
        delete b;
        return nullptr;
    }
    if ((st = api->CreateSessionOptions(&b->ctx.opts))) {
        set_ort_err(err, err_len, api, st, "CreateSessionOptions");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->SetIntraOpNumThreads(b->ctx.opts, 1))) {
        set_ort_err(err, err_len, api, st, "SetIntraOpNumThreads");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->SetSessionGraphOptimizationLevel(b->ctx.opts, ORT_ENABLE_ALL))) {
        set_ort_err(err, err_len, api, st, "SetSessionGraphOptimizationLevel");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->CreateSession(b->ctx.env, model_path, b->ctx.opts, &b->ctx.session))) {
        set_ort_err(err, err_len, api, st, "CreateSession");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if ((st = api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &b->ctx.mem))) {
        set_ort_err(err, err_len, api, st, "CreateCpuMemoryInfo");
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    if (!describe(api, b, err, err_len, "模型")) {
        ds_ort_destroy(reinterpret_cast<ds_backend*>(b));
        return nullptr;
    }
    b->be.ctx = b;
    b->be.onnx_run = &ort_run;
    return &b->be;
}

void ds_ort_destroy(ds_backend* be) {
    if (!be) return;
    auto* b = reinterpret_cast<Backend*>(be);
    const OrtApi* api = b->ctx.api;
    if (api) {
        for (auto& c : b->slots) {
            if (c.mem) api->ReleaseMemoryInfo(c.mem);
            if (c.session) api->ReleaseSession(c.session);
        }
        if (b->ctx.mem) api->ReleaseMemoryInfo(b->ctx.mem);
        if (b->ctx.session) api->ReleaseSession(b->ctx.session);
        if (b->ctx.opts) api->ReleaseSessionOptions(b->ctx.opts);
        if (b->ctx.env) api->ReleaseEnv(b->ctx.env);
    }
    delete b;
}

int64_t ds_ort_input_len(const ds_backend* be) {
    if (!be) return 0;
    const auto* b = reinterpret_cast<const Backend*>(be);
    return b->slots.empty() ? b->ctx.in_len : b->slots[0].in_len;
}

int64_t ds_ort_output_len(const ds_backend* be) {
    if (!be) return 0;
    const auto* b = reinterpret_cast<const Backend*>(be);
    return b->slots.empty() ? b->ctx.out_len : b->slots[0].out_len;
}

}  // extern "C"
