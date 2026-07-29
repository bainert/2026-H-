### Task 3: huidu.h — 导出 huidu_value 数组

**Files:**
- Modify: `ucdriver/huidu/huidu.h`

**接口:**
- Produces: `huidu_value[5]` extern 声明（供 motor.c 和 main.c 读取传感器状态）

- [ ] **Step 1: 添加 extern 声明**

在 `extern volatile uint8_t huidu_straight_count;` 后面添加：

```c
extern uint8_t huidu_value[5];
```
