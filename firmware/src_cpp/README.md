# src_cpp/ — 制御4モジュールのC++版（kanato が書く）

`src/` のC版は参考例（リファレンス）として残す。ここにはオーナー（kanato）が自分で書くC++版だけを置く。
このディレクトリは、空だとgitが保持しないために README を置いている。

## 順番

`thrust_model` → `mixer` → `controller_pid` → `attitude`（1つずつ。ファイル名は `src_cpp/<module>.cpp`）

## 切り替え方

モジュールごとに独立して切り替えられる。使える名前はこの4つだけ。

| 場面 | 指定 |
| --- | --- |
| Pico（CMake） | `-DFC_CPP_MODULES="thrust_model;mixer"` |
| ホスト試験 | `make -C firmware/test CPP_MODULES="thrust_model mixer"` |

- 指定したモジュールは `src/<m>.c` の代わりに `src_cpp/<m>.cpp` がビルドされる。他はCのまま。
- 全部C++なら `FC_CPP_MODULES="thrust_model;mixer;controller_pid;attitude"`（以前の案 `FC_CONTROL_LANG=CPP` 相当）。
- 名前が違う、または `.cpp` が無いとエラーで止まる。
- `make -C firmware/test` は `cxxheaders` で、ヘッダがC++から使えることを毎回確かめる。

## インターフェース

- 実装するのは `firmware/include/<m>.h` の関数。意味は変えない（ヘッダは変更しない）。
- ヘッダに `extern "C"` が入っているので、`.cpp` の先頭で**そのモジュールのヘッダを最初に** include すれば、定義は自動でCリンケージになる。
- `ipc_flags.h` `spsc_ring.h` `ipc.h` `control_loop.h` は C専用（`<stdatomic.h>`）。C++から include しない（4モジュールは依存しない）。

## コンパイル規則

コンパイルオプションは `-fno-exceptions -fno-rtti -Wall -Wextra -Wdouble-promotion -Wfloat-conversion`（ホスト試験は `-Werror` つき）。

提案（以前の議論＝JSF AV / Real-Time C++ 流に沿った選択。変えるときは相談）:

- 規格は C++17。
- 使わない: 例外、RTTI、`new` / `delete` / `malloc`、`std::vector` / `std::string` / iostream、`virtual`。
- floatの定数は `1.0f` と書く（`double` に昇格させない）。`std::sqrt` などは float に対して呼ぶ。
- `static_assert` と `constexpr` は歓迎。
