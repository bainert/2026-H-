### Task 1: key.h — 新增 lap 状态变量声明

**Files:**
- Modify: `ucdriver/key/key.h`

**接口:**
- Produces: `lap_state_t` 枚举类型, `lap_state`, `lap_start_ms`, `lap_total_ms` extern 声明

- [ ] **Step 1: 在 key.h 中添加枚举和 extern 声明**

修改 `ucdriver/key/key.h`，在 `#include` 下方、`extern volatile uint8_t key_debounce_cnt;` 后面添加：

```c
typedef enum {
    LAP_IDLE = 0,      // 待命
    LAP_RUNNING,       // 行驶中
    LAP_COMPLETE       // 完成一圈回到A点
} lap_state_t;

extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
```
