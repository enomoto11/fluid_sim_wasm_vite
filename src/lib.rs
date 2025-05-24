// src/lib.rs

use wasm_bindgen::prelude::*;

// 定数 (格子サイズ、速度、緩和時間など)
const NX: usize = 200; // X方向の格子点数
const NY: usize = 100; // Y方向の格子点数
const Q: usize = 9; // 速度の方向数 (D2Q9 モデル)

// 各方向の速度ベクトル (x, y)
const CX: [i32; Q] = [0, 1, 0, -1, 0, 1, -1, -1, 1];
const CY: [i32; Q] = [0, 0, 1, 0, -1, 1, 1, -1, -1];

// 重み
const W: [f64; Q] = [
    4.0 / 9.0,
    1.0 / 9.0,
    1.0 / 9.0,
    1.0 / 9.0,
    1.0 / 9.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
];

const OMEGA: f64 = 1.0; // 緩和時間 (tau = 1 / OMEGA, 通常 0.5 < tau < 2)

// シミュレーションの状態を保持する構造体
#[wasm_bindgen]
pub struct FluidSim {
    f: Vec<f64>,             // 現在の分布関数 (f[y * NX * Q + x * Q + q])
    f_temp: Vec<f64>,        // ストリーミング用の一時的な分布関数
    rho: Vec<f64>,           // 密度
    ux: Vec<f64>,            // X方向速度
    uy: Vec<f64>,            // Y方向速度
    obstacle_map: Vec<bool>, // 障害物マップ
    cylinder_radius: f64,
    cylinder_center_x: f64,
    cylinder_center_y: f64,
}

#[wasm_bindgen]
impl FluidSim {
    #[wasm_bindgen(constructor)]
    pub fn new(cylinder_radius: f64, cylinder_center_x: f64, cylinder_center_y: f64) -> FluidSim {
        let mut sim = FluidSim {
            f: vec![0.0; NX * NY * Q],
            f_temp: vec![0.0; NX * NY * Q],
            rho: vec![0.0; NX * NY],
            ux: vec![0.0; NX * NY],
            uy: vec![0.0; NX * NY],
            obstacle_map: vec![false; NX * NY],
            cylinder_radius,
            cylinder_center_x,
            cylinder_center_y,
        };

        // 初期化処理
        sim.init();
        sim
    }

    fn init(&mut self) {
        // 円柱の配置
        for y in 0..NY {
            for x in 0..NX {
                let dist_sq = (x as f64 - self.cylinder_center_x).powi(2)
                    + (y as f64 - self.cylinder_center_y).powi(2);
                if dist_sq < self.cylinder_radius.powi(2) {
                    self.obstacle_map[y * NX + x] = true;
                }
            }
        }

        // 初期速度と平衡分布の設定
        // より物理的な初期条件: 円柱の上流側でより速い流れ
        for y in 0..NY {
            for x in 0..NX {
                let initial_ux = if x < self.cylinder_center_x as usize {
                    0.08 // 上流側でより速い流れ
                } else {
                    0.02 // 下流側は遅い流れ
                };
                let initial_uy = 0.0;

                // 平衡分布f_eqを計算
                let mut feq_sum = 0.0;
                for q in 0..Q {
                    let c_dot_u = (CX[q] as f64) * initial_ux + (CY[q] as f64) * initial_uy;
                    let u_sq = initial_ux.powi(2) + initial_uy.powi(2);
                    let feq = W[q] * (1.0 + 3.0 * c_dot_u + 4.5 * c_dot_u.powi(2) - 1.5 * u_sq);
                    self.f[y * NX * Q + x * Q + q] = feq;
                    feq_sum += feq;
                }
                self.rho[y * NX + x] = feq_sum; // 初期密度は1.0に近い
                self.ux[y * NX + x] = initial_ux;
                self.uy[y * NX + x] = initial_uy;
            }
        }
    }

    #[wasm_bindgen]
    pub fn step(&mut self) {
        // 1. ストリーミング (Propagation) - 水平方向は周期境界
        for y in 0..NY {
            for x in 0..NX {
                for q in 0..Q {
                    // 水平方向（左右）は周期境界条件
                    let prev_x = (x as i32 - CX[q] + NX as i32) % NX as i32;
                    // 鉛直方向（上下）は後で壁境界として別途処理するため、ここでは境界チェック
                    let prev_y = y as i32 - CY[q];

                    // 上下境界をチェック
                    if prev_y < 0 || prev_y >= NY as i32 {
                        continue; // 上下境界は後で別途処理
                    }

                    self.f_temp[y * NX * Q + x * Q + q] =
                        self.f[(prev_y as usize) * NX * Q + (prev_x as usize) * Q + q];
                }
            }
        }
        self.f.copy_from_slice(&self.f_temp); // f_tempの内容をfにコピー

        // 2. 衝突 (Collision) とマクロ量計算
        for y in 0..NY {
            for x in 0..NX {
                let idx = y * NX + x;

                // 障害物の場合、何もしない (後で別途処理)
                if self.obstacle_map[idx] {
                    continue;
                }

                // 密度と速度の計算 (マクロ量)
                let mut rho_val = 0.0;
                let mut ux_val = 0.0;
                let mut uy_val = 0.0;

                for q in 0..Q {
                    let f_val = self.f[idx * Q + q];
                    rho_val += f_val;
                    ux_val += (CX[q] as f64) * f_val;
                    uy_val += (CY[q] as f64) * f_val;
                }
                self.rho[idx] = rho_val;
                self.ux[idx] = ux_val / rho_val;
                self.uy[idx] = uy_val / rho_val;

                // 平衡分布 f_eq の計算
                for q in 0..Q {
                    let feq = W[q]
                        * (rho_val
                            + 3.0
                                * ((CX[q] as f64) * self.ux[idx] + (CY[q] as f64) * self.uy[idx])
                            + 4.5
                                * ((CX[q] as f64) * self.ux[idx] + (CY[q] as f64) * self.uy[idx])
                                    .powi(2)
                            - 1.5 * (self.ux[idx].powi(2) + self.uy[idx].powi(2)));
                    // 衝突項
                    self.f[idx * Q + q] = self.f[idx * Q + q] - OMEGA * (self.f[idx * Q + q] - feq);
                }
            }
        }

        // 3. 境界条件

        // 上下境界: スリップ壁境界条件（摩擦なし）
        // 上壁 (y = NY-1)
        for x in 0..NX {
            let y = NY - 1;
            let idx = y * NX + x;
            if self.obstacle_map[idx] {
                continue;
            }

            // スリップ境界: y方向速度を0にし、x方向速度は保持
            self.uy[idx] = 0.0;

            // 上向きの粒子を下向きに反射（bounce-back for y-component）
            // q=2 (+y) <-> q=4 (-y), q=5 (+x+y) <-> q=8 (+x-y), q=6 (-x+y) <-> q=7 (-x-y)
            let temp_f2 = self.f[idx * Q + 2];
            let temp_f5 = self.f[idx * Q + 5];
            let temp_f6 = self.f[idx * Q + 6];

            self.f[idx * Q + 4] = temp_f2; // +y -> -y
            self.f[idx * Q + 8] = temp_f5; // +x+y -> +x-y
            self.f[idx * Q + 7] = temp_f6; // -x+y -> -x-y
        }

        // 下壁 (y = 0)
        for x in 0..NX {
            let y = 0;
            let idx = y * NX + x;
            if self.obstacle_map[idx] {
                continue;
            }

            // スリップ境界: y方向速度を0にし、x方向速度は保持
            self.uy[idx] = 0.0;

            // 下向きの粒子を上向きに反射
            let temp_f4 = self.f[idx * Q + 4];
            let temp_f7 = self.f[idx * Q + 7];
            let temp_f8 = self.f[idx * Q + 8];

            self.f[idx * Q + 2] = temp_f4; // -y -> +y
            self.f[idx * Q + 5] = temp_f8; // +x-y -> +x+y
            self.f[idx * Q + 6] = temp_f7; // -x-y -> -x+y
        }

        // 障害物境界 (Bounce-back)
        for y in 0..NY {
            for x in 0..NX {
                let idx = y * NX + x;
                if self.obstacle_map[idx] {
                    // 障害物セルの場合、衝突した粒子の方向を反転させる (Bounce-back)
                    for q in 0..Q {
                        let opp_q = match q {
                            // 反対方向のインデックス
                            0 => 0,
                            1 => 3, // +x -> -x
                            2 => 4, // +y -> -y
                            3 => 1, // -x -> +x
                            4 => 2, // -y -> +y
                            5 => 7, // +x+y -> -x-y
                            6 => 8, // -x+y -> +x-y
                            7 => 5, // -x-y -> +x+y
                            8 => 6, // +x-y -> -x+y
                            _ => unreachable!(),
                        };
                        self.f[idx * Q + opp_q] = self.f_temp[idx * Q + q]; // ストリーミング前の値をコピー
                    }
                }
            }
        }
    }

    #[wasm_bindgen]
    pub fn get_velocity_magnitudes(&self) -> Vec<f64> {
        self.ux
            .iter()
            .zip(self.uy.iter())
            .map(|(&u_x, &u_y)| (u_x.powi(2) + u_y.powi(2)).sqrt())
            .collect()
    }

    #[wasm_bindgen]
    pub fn get_nx(&self) -> usize {
        NX
    }

    #[wasm_bindgen]
    pub fn get_ny(&self) -> usize {
        NY
    }

    #[wasm_bindgen]
    pub fn get_obstacle_map(&self) -> Vec<u8> {
        self.obstacle_map
            .iter()
            .map(|&b| if b { 1 } else { 0 })
            .collect()
    }
}
