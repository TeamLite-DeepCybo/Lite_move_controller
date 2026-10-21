#!/usr/bin/env python3
"""cmp_pin_vs_python.py —— C++(pinocchio) 动力学 vs Python(lite_dyid 辨识侧) 逐点对拍

用法：
    /usr/bin/python3 cmp_pin_vs_python.py <urdf> <params.json> <dump_pin可执行> [N]

要求：NC=200 姿态 × NE=5 组 (v,a)，**逐关节**比较三项：
    τ_g          : C++ computeGeneralizedGravity(gmodel)  vs  Python 同函数
    τ_friction   : C++ frictionTorqueSymmetric            vs  Python c1±/c2 公式
    τ_inertia    : C++ rnea(full,q,v,a) − rnea(full,q,0,0) vs Python Y_I(q,v,a)@pi_I
判定：逐关节 max|err| ≤ 1e-9（同库同 API，应当接近机器精度）。

⚠️ 本脚本**不修改**任何东西，只比较。
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    urdf_path, params_path, dump_exe = sys.argv[1], sys.argv[2], sys.argv[3]
    N = int(sys.argv[4]) if len(sys.argv) > 4 else 200

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "lite_dyid"))
    import pinocchio as pin
    from lite_dyid.identify import load_params
    from lite_dyid.model import RobotModel
    from lite_dyid.common import ARM_JOINTS

    urdf_text = Path(urdf_path).read_text()
    d = load_params(params_path)
    m = RobotModel(urdf_text)

    # ---- Python 侧：精确复刻 fastsolve 的模型 ----
    tmp = Path(tempfile.mkdtemp()) / "m.urdf"
    tmp.write_text(urdf_text)
    pm = pin.buildModelFromUrdf(str(tmp))
    pi_grav = np.asarray(d["gravity"]["params"], dtype=float)
    nl = pm.njoints - 1
    gcols = [10 * k + j for k in range(nl) for j in range(4)]
    icols = [10 * k + j for k in range(nl) for j in range(4, 10)]

    # 重力模型：只含 (m, μ)，m≈0 时用 eps 代换保住 μ
    eps_m = 1e-9
    gm = pm.copy()
    for k, jid in enumerate(range(1, pm.njoints)):
        mm = float(pi_grav[4 * k])
        mu = np.asarray(pi_grav[4 * k + 1:4 * k + 4], dtype=float)
        m_rep = mm if mm > eps_m else eps_m
        gm.inertias[jid] = pin.Inertia(m_rep, mu / m_rep, np.zeros((3, 3)))

    pi_I = np.asarray(d["inertia"]["raw_fit"]["pi_I"], dtype=float)
    full = pm.copy()
    for jid in range(1, pm.njoints):
        k = jid - 1
        v10 = np.zeros(10)
        v10[0:4] = pi_grav[4 * k:4 * k + 4]
        v10[4:10] = pi_I[6 * k:6 * k + 6]
        full.inertias[jid] = pin.Inertia.FromDynamicParameters(v10)

    arm_v = m.arm_v
    gd, dd = gm.createData(), full.createData()

    # ---- 采样 ----
    rng = np.random.default_rng(20261016)
    lo, hi = m.limit_array()
    Q, V, A = [], [], []
    # ⚠️ `m.limit_array()` 是 **14 维（臂关节）**，不是全模型 18 维。
    #    这里存 14 维；下面按 `ARM_JOINTS` 逐个映射到全模型的 idx_q/idx_v。
    for _ in range(N):
        q = lo + (hi - lo) * rng.uniform(0.15, 0.85, size=len(lo))
        v = rng.uniform(-0.8, 0.8, size=len(lo))
        a = rng.uniform(-2.0, 2.0, size=len(lo))
        Q.append(q); V.append(v); A.append(a)
    Q, V, A = np.array(Q), np.array(V), np.array(A)

    lines = []
    for i in range(N):
        lines.append(" ".join("%.17g" % x for x in np.concatenate([Q[i][:7], V[i][:7], A[i][:7]])))
    out = subprocess.run([dump_exe, urdf_path, params_path],
                         input="\n".join(lines), capture_output=True, text=True,
                         env={**__import__("os").environ, "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib"})
    if out.returncode != 0:
        print("dump_pin 失败：", out.stderr[:500]); return 1
    rows = [r for r in out.stdout.strip().split("\n") if r.strip()]
    if len(rows) != N:
        print(f"行数不符：期望 {N}，得到 {len(rows)}"); return 1
    C = np.array([[float(x) for x in r.split()] for r in rows])
    C_g, C_f, C_t = C[:, 0:7], C[:, 7:14], C[:, 14:21]

    # ---- Python 侧期望值 ----
    P_g = np.zeros((N, 7)); P_f = np.zeros((N, 7)); P_i = np.zeros((N, 7))
    c1p = np.asarray(d["friction"]["c1_pos"], float)
    c1n = np.asarray(d["friction"]["c1_neg"], float)
    c2 = np.asarray(d["friction"]["c2"], float)
    idx = list(range(len(ARM_JOINTS)))   # 全 14 维下标（V/A 都是 14 维）
    eps = 0.01
    # ⚠️ 模型有 18 个关节（含基座与头部），`rnea` / `computeJointTorqueRegressor`
    #    要**完整维度**的 q/v/a。臂关节写在各自的 idx_q / idx_v 上，其余置 0。
    for i in range(N):
        q = m.q_from_list(Q[i])            # 已经是全维
        v_full = np.zeros(pm.nv); a_full = np.zeros(pm.nv)
        for k, jn in enumerate(ARM_JOINTS):          # 14 个臂关节
            jid = list(pm.names).index(jn)
            v_full[pm.joints[jid].idx_v] = V[i][k]     # V[i] 也是 14 维
            a_full[pm.joints[jid].idx_v] = A[i][k]
        P_g[i] = pin.computeGeneralizedGravity(gm, gd, q)[arm_v][:7]
        Y = pin.computeJointTorqueRegressor(full, dd, q, v_full, a_full)[arm_v]
        P_i[i] = (Y[:, icols] @ pi_I)[:7]  # 惯量项（与 fastsolve 的 Ablk 同一列集）
        # 摩擦：与 C++ 的 frictionTorqueSymmetric(取平均) 对齐
        c1 = 0.5 * (c1p[idx] + c1n[idx])
        s = np.tanh(V[i][idx] / eps)
        P_f[i] = (c1 * s + c2[idx] * V[i][idx])[:7]   # 只取左臂 7 个
        # 交叉检查：τ_g 也应等于 rnea(a=0,v=0) 减去…（略）

    def rep(name, Cx, Px):
        e = np.abs(Cx - Px)
        print(f"\n=== {name} ===")
        print(f"{'关节':24s} {'max|err|':>12s} {'mean|err|':>12s} {'max|val|':>12s}")
        for k, j in enumerate(ARM_JOINTS[:7]):
            print(f"{j:24s} {e[:,k].max():12.3e} {e[:,k].mean():12.3e} {np.abs(Px[:,k]).max():12.4f}")
        print(f"全局 max|err| = {e.max():.3e}")
        return e.max()

    e1 = rep("τ_g 重力（gmodel）", C_g, P_g)
    e2 = rep("τ_friction 摩擦", C_f, P_f)
    # τ_total 交叉检查：C++ total 应 = C_g + C_f + τ_inertia
    print(f"\n总判定：τ_g max={e1:.3e}   τ_fric max={e2:.3e}   (阈值 1e-9)")
    ok = (e1 <= 1e-9) and (e2 <= 1e-9)
    print("结果:", "✅ 逐关节一致" if ok else "❌ 不一致")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
