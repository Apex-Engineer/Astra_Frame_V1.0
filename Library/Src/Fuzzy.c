#include "Fuzzy.h"

/* ============================================================
 *  第一部分: 隶属度函数 — 一个数字属于某个词的"程度"
 *
 *  7 个词: NB NM NS ZO PS PM PB
 *  每个词用三角形表示, 三角形有 3 个参数: [左底, 顶点, 右底]
 *
 *          μ=1  ▵
 *              /|\
 *             / | \
 *            /  |  \
 *           /   |   \
 *          /    |    \
 *       ──┴────┴────┴──→ x
 *          a    b    c
 *
 *  在 [a,b] 段: μ = (x-a)/(b-a)   上升边
 *  在 [b,c] 段: μ = (c-x)/(c-b)   下降边
 *  在 [a,c] 外: μ = 0
 * ============================================================ */

// 7个模糊集的三角形参数 [左底a, 顶点b, 右底c]
static const float MF_PARAM[7][3] = {
    {-6, -6, -4}, // NB: 左开, 三角形只有右半边
    {-6, -4, -2}, // NM: 完整三角形
    {-4, -2, 0},  // NS
    {-2, 0, 2},   // ZO
    {0, 2, 4},    // PS
    {2, 4, 6},    // PM
    {4, 6, 6},    // PB: 右开, 三角形只有左半边
};

// 计算 x 在模糊集 set 上的隶属度 (0.0 ~ 1.0)
static float membership(float x, int set)
{
    float a = MF_PARAM[set][0];
    float b = MF_PARAM[set][1];
    float c = MF_PARAM[set][2];

    if (x <= a || x >= c)
        return 0.0f; // 完全不属于
    if (x <= b)
        return (x - a) / (b - a); // 上升边
    else
        return (c - x) / (c - b); // 下降边
}

/* ============================================================
 *  第二部分: 规则表 — 这是整个模糊控制器的"大脑"
 *
 *  规则形式: IF E is A AND EC is B THEN U is C
 *  输入 E (行) × 输入 EC (列) → 输出 U
 *
 *  rule_table[E_set][EC_set] = U_set
 *
 *  比如 rule_table[PB][NB] = ZO
 *     → 误差正大(超调很严重) + 误差变化负大(在快速回落) → 输出零(别动了, 等它自己回来)
 *
 *        EC: NB  NM  NS  ZO  PS  PM  PB
 *  E  ┌─────────────────────────────────
 *  NB │    PB  PB  PM  PM  PS  ZO  ZO   误差负大(太低)
 *  NM │    PB  PM  PM  PS  PS  ZO  NS
 *  NS │    PM  PM  PS  PS  ZO  NS  NS
 *  ZO │    PM  PS  PS  ZO  NS  NS  NM
 *  PS │    PS  PS  ZO  NS  NS  NM  NM   误差正小(稍高)
 *  PM │    PS  ZO  NS  NS  NM  NM  NB
 *  PB │    ZO  ZO  NS  NM  NM  NB  NB   误差正大(太高)
 *
 *  总体规律: 左上角 → 右下角, 输出从 PB 渐变到 NB
 *            误差为正(超调)就给负输出(减小), 误差为负就给正输出(增大)
 *            变化率也会影响: 变化率方向不同, 输出会相应调整
 * ============================================================ */
static const int RULE_TABLE[7][7] = {
    // EC: NB  NM  NS  ZO  PS  PM  PB     E:
    {6, 6, 5, 5, 4, 3, 3}, // NB (输出PB=6, PM=5, PS=4, ZO=3)
    {6, 5, 5, 4, 4, 3, 2}, // NM
    {5, 5, 4, 4, 3, 2, 2}, // NS
    {5, 4, 4, 3, 2, 2, 1}, // ZO
    {4, 4, 3, 2, 2, 1, 1}, // PS
    {4, 3, 2, 2, 1, 1, 0}, // PM
    {3, 3, 2, 1, 1, 0, 0}, // PB (输出NB=0)
};

// 每个模糊集的中心值 (用于去模糊化)
static const float CENTER[7] = {-6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f};

static float fuzzy_inference(float e, float ec, const int rule_table[FUZZY_SETS][FUZZY_SETS])
{
    /* ========== 阶段 A: 模糊推理 ==========
     *
     * 目标: 找出哪些规则被触发, 每条规则被触发了多少
     * 结果存在 output_mu[] 数组里, 每个词一个值
     *
     * 比如 output_mu[2] = 0.5 意思是:
     *   输出词"NS(负小)"的确定程度是 50%
     */

    float output_mu[7] = {0.0f}; // 7个输出词的隶属度, 全部初始化为0

    /* 遍历全部 7×7 = 49 条规则 */
    for (int i = 0; i < 7; i++) // i = E 属于哪个词 (行)
    {
        float mu_e = membership(e, i); // E 对第 i 个词的隶属度//只对前三个词有隶属度？
        if (mu_e == 0.0f)
            continue; // E 跟这个词没关系, 跳过这一整行

        for (int j = 0; j < 7; j++) // j = EC 属于哪个词 (列)
        {
            float mu_ec = membership(ec, j); // EC 对第 j 个词的隶属度
            if (mu_ec == 0.0f)
                continue; // EC 跟这个词没关系, 跳过

            /* ---- 关键: 规则激活度 = min(μ_E, μ_EC) ----
             *
             * 为什么用 min? 因为规则是 E AND EC
             * "又冷又饿" 的忍受程度取决于更糟的那个
             * 冷度 70%, 饿度 30% → 整体难受程度 30%
             * 如果求和或求平均, 就会高估触发程度
             */
            float activation = (mu_e < mu_ec) ? mu_e : mu_ec;

            // 查规则表: E=i, EC=j → U=?
            int u_set = rule_table[i][j];

            /* ---- 关键: 多条规则可能指向同一个输出词 ----
             *
             * 比如:
             *   规则A: E=NS, EC=ZO → U=NS, 激活度 0.4
             *   规则B: E=ZO, EC=NS → U=NS, 激活度 0.6
             * 两条规则都说"输出应该负小", 取最大值 0.6
             *
             * 这是取最大(max)不是取最小(min),
             * 因为两条规则是"或者"关系, 谁更确定信谁
             */
            if (activation > output_mu[u_set])
            {
                output_mu[u_set] = activation;
            }
        }
    }

    /* ========== 阶段 B: 去模糊化 — 重心法 ==========
     *
     * 此时 output_mu[] 长这样 (举例):
     *   output_mu[NS] = 0.6   ← 60% 确定要"负小"
     *   output_mu[ZO] = 0.4   ← 40% 确定要"零"
     *   其余 = 0
     *
     * 问题: 不能输出 "负小" 这个字, 得输出一个数字!
     *
     * 解决: 重心法 — 把每个词的中心位置加权平均
     *
     *          μ
     *         1.0┤
     *            │
     *         0.6┤     ┌──┐         ← NS 中心 -2, 高度 0.6
     *            │     │  │
     *         0.4┤     │  ├──┐      ← ZO 中心  0, 高度 0.4
     *            │     │  │  │
     *           0├─────┼──┼──┼──→ U
     *                 -2    0
     *
     *   重心 = (-2 × 0.6 + 0 × 0.4) / (0.6 + 0.4)
     *        = (-1.2 + 0) / 1.0
     *        = -1.2
     *
     *   → 输出 -1.2, 偏向 NS(负小), 但被 ZO 往回拉了一点
     *
     * 直观理解: 每个词都在"投票", 置信度越高票数越多,
     *           最终位置是所有投票的加权平均值
     */

    float numerator = 0.0f;   // 分子: Σ (隶属度 × 中心值)
    float denominator = 0.0f; // 分母: Σ (隶属度)

    for (int k = 0; k < 7; k++)
    {
        numerator += output_mu[k] * CENTER[k];
        denominator += output_mu[k];
    }

    if (denominator < 0.0001f)
        return 0.0f; // 无规则激活, 返回 0

    return numerator / denominator;
}

/**
 * @brief  初始化模糊控制器
 * @param  fuzzy    控制器结构体
 * @param  Ke       误差量化因子 (物理值 → 模糊论域)
 * @param  Kec      误差变化量化因子
 * @param  Ku       输出比例因子 (模糊论域 → 物理值)
 * @param  out_max  输出上限
 *
 *      量化因子
 *      假如误差物理范围是 ±50°C, 模糊论域是 ±6
 *      则 Ke = 6 / 50 = 0.12
 *         kec 同理, 取决于误差变化率的物理范围
 *      假如模糊输出 ±6 对应实际输出 ±10000 (PWM)
 *      则 Ku = 10000 / 6 = 1666.7
 */
void Fuzzy_init(fuzzy_type_def *fuzzy, float Ke, float Kec, float Ku, float out_max)
{
    fuzzy->Ke = Ke;
    fuzzy->Kec = Kec;
    fuzzy->Ku = Ku;
    fuzzy->out_max = out_max;
    fuzzy->rule_table = RULE_TABLE; // 默认使用内置规则表
    fuzzy->error = 0.0f;
    fuzzy->last_error = 0.0f;
    fuzzy->out = 0.0f;
}

/**
 * @brief  初始化模糊控制器 (使用自定义规则表)
 */
void Fuzzy_init_ex(fuzzy_type_def *fuzzy, float Ke, float Kec, float Ku, float out_max,
                   const int rule_table[FUZZY_SETS][FUZZY_SETS])
{
    Fuzzy_init(fuzzy, Ke, Kec, Ku, out_max);
    fuzzy->rule_table = rule_table;
}

/**
 * @brief  执行一次模糊控制计算
 * @param  fuzzy  控制器结构体
 * @param  ref    实际值 (传感器读数)
 * @param  set    设定值 (目标值)
 * @return        精确控制量 (直接写给执行器)
 */
float Fuzzy_Calc(fuzzy_type_def *fuzzy, float ref, float set)
{
    /* --- 1. 计算误差和误差变化率 --- */
    fuzzy->error = set - ref;
    fuzzy->error_change = fuzzy->error - fuzzy->last_error;
    fuzzy->last_error = fuzzy->error;

    /* --- 2. 量化: 物理值 → 模糊论域 [-6, 6] --- */
    float e = fuzzy->error * fuzzy->Ke;
    float ec = fuzzy->error_change * fuzzy->Kec;

    // 限幅到模糊论域
    if (e > FUZZY_MAX)
        e = FUZZY_MAX;
    if (e < -FUZZY_MAX)
        e = -FUZZY_MAX;
    if (ec > FUZZY_MAX)
        ec = FUZZY_MAX;
    if (ec < -FUZZY_MAX)
        ec = -FUZZY_MAX;

    /* --- 3. 模糊推理 + 去模糊化 --- */
    float u_fuzzy = fuzzy_inference(e, ec, fuzzy->rule_table); // 结果在 [-6, 6] 范围

    /* --- 4. 反量化: 模糊论域 → 物理输出 --- */
    fuzzy->out = u_fuzzy * fuzzy->Ku;

    /* --- 5. 输出限幅 --- */
    if (fuzzy->out > fuzzy->out_max)
        fuzzy->out = fuzzy->out_max;
    if (fuzzy->out < -fuzzy->out_max)
        fuzzy->out = -fuzzy->out_max;

    return fuzzy->out;
}

/**
 * @brief  直接输入 e, ec 进行模糊推理 (不更新内部误差状态, 供模糊 PID 复用)
 * @param  fuzzy  控制器结构体
 * @param  e      误差 (已量化的物理值, 函数内部会限幅到 [-6,6])
 * @param  ec     误差变化率 (已量化的物理值)
 * @return        精确输出量
 */
float Fuzzy_Calc_Raw(fuzzy_type_def *fuzzy, float e, float ec)
{
    // 限幅到模糊论域
    if (e > FUZZY_MAX)
        e = FUZZY_MAX;
    if (e < -FUZZY_MAX)
        e = -FUZZY_MAX;
    if (ec > FUZZY_MAX)
        ec = FUZZY_MAX;
    if (ec < -FUZZY_MAX)
        ec = -FUZZY_MAX;

    // 模糊推理 + 去模糊化, 结果在 [-6, 6] 范围
    float u_fuzzy = fuzzy_inference(e, ec, fuzzy->rule_table);

    // 反量化并限幅
    float out = u_fuzzy * fuzzy->Ku;
    if (out > fuzzy->out_max)
        out = fuzzy->out_max;
    if (out < -fuzzy->out_max)
        out = -fuzzy->out_max;

    return out;
}
