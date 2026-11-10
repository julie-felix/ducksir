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
    std::string in_name;
    std::string out_name;
    int64_t in_len = 0;
    int64_t out_len = 0;
    std::vector<int64_t> in_dims;   // 模型要的真实形状（如 [1,34]）：张量必须按它建，不能摊成一维
    std::vector<int64_t> out_dims;
};

struct Backend {
    ds_backend be{};
    OrtCtx ctx{};
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
    OrtStatus* st = nullptr;
    size_t n_in = 0, n_out = 0;
    if ((st = api->SessionGetInputCount(b->ctx.session, &n_in))) {
        set_ort_err(err, err_len, api, st, "SessionGetInputCount");
        return false;
    }
    if ((st = api->SessionGetOutputCount(b->ctx.session, &n_out))) {
        set_ort_err(err, err_len, api, st, "SessionGetOutputCount");
        return false;
    }
    if (n_in != 1 || n_out != 1) {
        std::snprintf(err, err_len, "%s 必须是单输入单输出（实际 %zu/%zu）", what, n_in, n_out);
        return false;
    }

    OrtAllocator* alloc = nullptr;
    if ((st = api->GetAllocatorWithDefaultOptions(&alloc))) {
        set_ort_err(err, err_len, api, st, "GetAllocatorWithDefaultOptions");
        return false;
    }
    char* name = nullptr;
    if ((st = api->SessionGetInputName(b->ctx.session, 0, alloc, &name))) {
        set_ort_err(err, err_len, api, st, "SessionGetInputName");
        return false;
    }
    b->ctx.in_name = name;
    if ((st = api->AllocatorFree(alloc, name))) {
        set_ort_err(err, err_len, api, st, "AllocatorFree");
        return false;
    }
    if ((st = api->SessionGetOutputName(b->ctx.session, 0, alloc, &name))) {
        set_ort_err(err, err_len, api, st, "SessionGetOutputName");
        return false;
    }
    b->ctx.out_name = name;
    if ((st = api->AllocatorFree(alloc, name))) {
        set_ort_err(err, err_len, api, st, "AllocatorFree");
        return false;
    }

    auto shape_of = [&](int is_input, int64_t* len, std::vector<int64_t>* keep) -> bool {
        OrtTypeInfo* info = nullptr;
        OrtStatus* s2 = is_input ? api->SessionGetInputTypeInfo(b->ctx.session, 0, &info)
                                 : api->SessionGetOutputTypeInfo(b->ctx.session, 0, &info);
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
                std::snprintf(err, err_len, "%s 有动态维度（%s），本核要求静态形状",
                              is_input ? "输入" : "输出", is_input ? b->ctx.in_name.c_str() : b->ctx.out_name.c_str());
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
        // 单 batch 时也保持原形状（ORT 对 rank 敏感）；只有纯一维才用 [total]
        keep->assign(dims.begin(), dims.end());
        api->ReleaseTypeInfo(info);
        return true;
    };

    if (!shape_of(1, &b->ctx.in_len, &b->ctx.in_dims)) return false;
    if (!shape_of(0, &b->ctx.out_len, &b->ctx.out_dims)) return false;
    return true;
}

int ort_run(void* raw_ctx, int model_id, const float* in, int in_len, float* out, int out_len) {
    (void)model_id;  // 一个后端实例绑一份模型；多模型就建多个实例
    auto* b = static_cast<Backend*>(raw_ctx);
    if (!b || !in || !out) return 1;
    if (in_len != b->ctx.in_len || out_len != b->ctx.out_len) return 2;

    const OrtApi* api = b->ctx.api;
    OrtValue* input = nullptr;
    OrtStatus* st = api->CreateTensorWithDataAsOrtValue(
        b->ctx.mem, const_cast<float*>(in), static_cast<size_t>(in_len) * sizeof(float),
        b->ctx.in_dims.data(), b->ctx.in_dims.size(),
        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input);
    if (st) {
        api->ReleaseStatus(st);
        return 3;
    }
    const char* in_names[1] = {b->ctx.in_name.c_str()};
    const char* out_names[1] = {b->ctx.out_name.c_str()};
    OrtValue* output = nullptr;
    st = api->Run(b->ctx.session, nullptr, in_names, &input, 1, out_names, 1, &output);
    api->ReleaseValue(input);
    if (st) {
        api->ReleaseStatus(st);
        return 4;
    }
    float* data = nullptr;
    st = api->GetTensorMutableData(output, reinterpret_cast<void**>(&data));
    if (st) {
        api->ReleaseStatus(st);
        api->ReleaseValue(output);
        return 5;
    }
    std::memcpy(out, data, static_cast<size_t>(out_len) * sizeof(float));
    api->ReleaseValue(output);
    return 0;
}

}  // namespace

extern "C" {

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
        if (b->ctx.mem) api->ReleaseMemoryInfo(b->ctx.mem);
        if (b->ctx.session) api->ReleaseSession(b->ctx.session);
        if (b->ctx.opts) api->ReleaseSessionOptions(b->ctx.opts);
        if (b->ctx.env) api->ReleaseEnv(b->ctx.env);
    }
    delete b;
}

int64_t ds_ort_input_len(const ds_backend* be) {
    if (!be) return 0;
    return reinterpret_cast<const Backend*>(be)->ctx.in_len;
}

int64_t ds_ort_output_len(const ds_backend* be) {
    if (!be) return 0;
    return reinterpret_cast<const Backend*>(be)->ctx.out_len;
}

}  // extern "C"
