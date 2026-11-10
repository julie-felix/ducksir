/* ducksim 执行核的 C ABI。
 *
 * 这一层的全部职责：加载一张编译好的图（blob），每步吃 obs、吐 action。
 * 不碰物理、不碰总线、不知道 MuJoCo 存在（计划 D9）。
 *
 * 三条硬约束（D7）：
 * 1. 单文件实现、arena 分配：`ds_load` 里分配，`ds_step` 里一次 malloc 都没有；
 * 2. 零全局态：所有状态在 `ds_plan` 里，可以同时存在多份；
 * 3. 确定性：同一 blob + 同一 obs 序列，任何平台算出同一份 action（节点按声明顺序执行，
 *    定点偏移、不做并行归约，编译时关掉 FMA 收缩）。
 *
 * 版本策略：blob 头带 `version` / `abi` / `required`（节点类型 + 最低版本）。
 * 老核遇到不认识的节点会**整份拒绝**（`ds_load` 返回 NULL），不会跑一半。
 */
#ifndef DUCKSIM_H
#define DUCKSIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DUCKSIM_ABI 1

/* 节点类型 id：与 sim/plan/ir.py 的 NODE_TYPES 一一对应 */
enum ds_node_type {
    DS_NODE_OBS = 1,
    DS_NODE_CONCAT = 2,
    DS_NODE_AFFINE = 3,
    DS_NODE_CLIP = 4,
    DS_NODE_REORDER = 5,
    DS_NODE_HISTORY = 6,
    DS_NODE_MODEL = 7,
    DS_NODE_OUT_ACTION = 8,
};

/* 节点 flag */
enum ds_node_flag {
    DS_FLAG_PASS_THROUGH = 1u << 0, /* 仅测试：model 节点不调后端，直接拷贝 */
    DS_FLAG_NATIVE_ONLY = 1u << 1,   /* 逃生口：出部署产物时必须拒绝 */
};

/* 返回码（负数是错，0 是成功） */
enum ds_status {
    DS_OK = 0,
    DS_ERR_NULL = -1,        /* 空指针 */
    DS_ERR_BAD_BLOB = -2,    /* 结构不合法（magic/版本/边界/顺序/digest） */
    DS_ERR_UNSUPPORTED = -3, /* 需要的节点类型或版本超出本核 */
    DS_ERR_NO_BACKEND = -4,  /* 有 model 节点但没给后端 */
    DS_ERR_GRAPH = -5,       /* 图上跑不通（比如没有 out_action） */
};

/* 推理后端：核不认识 ONNX，只认识这个回调（D8）。native 端会把它接到 onnxruntime。 */
typedef struct ds_backend {
    void* ctx;
    /* in/out 都是 float32；返回 0 成功，非 0 视为推理失败 */
    int (*onnx_run)(void* ctx, int model_id, const float* in, int in_len, float* out, int out_len);
} ds_backend;

typedef struct ds_plan ds_plan;

typedef struct ds_info {
    uint16_t version;
    uint16_t abi;
    uint16_t node_count;
    uint16_t input_dim;
    uint16_t output_dim;
    uint32_t arena_len; /* 图用到的 float 总数（一次性分配） */
    uint32_t param_len; /* 参数字符数 */
} ds_info;

/* 加载：成功返回 plan（调用方负责 ds_free），失败返回 NULL 并把原因写进 err。 */
ds_plan* ds_load(const void* blob, size_t len, char* err, size_t err_len);

void ds_free(ds_plan* plan);

int ds_info_of(const ds_plan* plan, ds_info* out);

/* 复位历史缓冲（history 节点），回到「刚上电」的状态。 */
int ds_reset(ds_plan* plan);

/* 跑一步：obs 长度必须是 info.input_dim，action 长度必须是 info.output_dim。 */
int ds_step(ds_plan* plan, const float* obs, float* action, const ds_backend* backend);

/* 本核支持的最高 ABI（给加载方做前置判断） */
uint16_t ds_abi_version(void);

#ifdef __cplusplus
}
#endif

#endif /* DUCKSIM_H */
