# 🌊 fluid_sim_wasm_vite

Rust + WebAssembly + Vite で動く、円柱まわり流れの流体シミュレーション

<img width="789" alt="image" src="https://github.com/user-attachments/assets/38ac2b7e-0839-418a-903e-7888bbebeda8" />


---

## 🚀 デモ

1. **ローカルで起動**
   ```bash
   npm install
   wasm-pack build --target web
   npm run dev
   ```
2. ブラウザで [http://localhost:5173/](http://localhost:5173/) を開く

---

## 🛠️ 技術スタック

- Rust (流体計算ロジック)
- wasm-bindgen / wasm-pack
- Vite (超高速フロントエンド開発サーバー)
- JavaScript (描画)

---

## 🌀 シミュレーション概要

- **モデル**: D2Q9 Lattice Boltzmann 法
- **障害物**: 円柱
- **可視化**: 速度場を色で表示

---

## 📁 ディレクトリ構成

```
fluid_sim_wasm_vite/
├── src/           # Rustソース
├── www/           # フロントエンド (index.html, pkg/)
├── www/pkg/       # wasm-packで生成されるWASM/JS
├── package.json   # Vite用
├── Cargo.toml     # Rust用
└── README.md
```

---

## 📝 ライセンス

MIT

---

## 🙌 Author

[enomoto11](https://github.com/enomoto11)
