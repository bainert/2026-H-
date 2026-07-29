### Task 2: key.c — 定义 lap 状态变量

**Files:**
- Modify: `ucdriver/key/key.c`

**接口:**
- Consumes: `lap_state_t` 枚举（来自 Task 1）
- Produces: `lap_state`, `lap_start_ms`, `lap_total_ms` 变量定义

- [ ] **Step 1: 在 key.c 末尾 static 变量定义区之后添加**

在 `volatile uint8_t key_debounce_cnt = 0;` 后面添加：

```c
// 第二问：跑圈状态
volatile lap_state_t lap_state = LAP_IDLE;
volatile uint32_t lap_start_ms = 0;
volatile uint32_t lap_total_ms = 0;
```
