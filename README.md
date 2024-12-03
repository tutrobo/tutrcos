# tutrcos

とよはし⭐︎ロボコンズ 新入生講習用ライブラリ

- STM32CubeF4 V1.28.1
- CMSIS-RTOS2 V2.1.0
- FreeRTOS Kernel V10.3.1

で開発・動作確認しています。

## ドキュメント

https://tutrobo.github.io/tutrcos/

## 導入方法

このライブラリには CMSIS-RTOS2(FreeRTOS) が必要です。

### Timebase Sourceの切り替え

RTOSを使用するにあたって Timebase Source を `SysTick` から任意のハードウェアタイマーに切り替えることが推奨されています。

`Pinout & Configuration` -> `System Core` -> `SYS` を選択し、Timebase Source に空いているハードウェアタイマーを指定してください(TIM6 が一般的)。

### CMSIS-RTOS2 の有効化、`main_thread` の設定

`Pinout & Configuration` -> `Middleware and Software Packs` -> `FREERTOS` を選択し、Interface に `CMSIS_V2` を指定します。

Configuration欄から `Advanced settings` を選択し、`USE_NEWLIB_REENTRANT` を `Enabled` にします。

Configuration欄から `Tasks and Queues` を選択し、`defaultTask` の

- Stack Size (Words): 1024
- Entry Function: `main_thread`
- Code Generation Option: As weak

に設定します。

### プロジェクトにライブラリを追加

Git submodule として導入するのがおすすめです。プロジェクトのルートで

```sh
$ git submodule add https://github.com/tutrobo/tutrcos.git
```

を実行します。

#### STM32 VS Code Extension ユーザーの場合

このライブラリは CMake に対応しています。STM32CubeMX を用いて CMake 向けにプロジェクトを書き出したあと、`Import CMake project` からプロジェクトを取り込み、`CMakeLists.txt` を以下のように書き換えます。

##### 変更前:

```cmake
# ~前略~

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
```

##### 変更後:

```cmake
# ~前略~

add_subdirectory(tutrcos)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    tutrcos
)
```

#### STM32CubeIDE ユーザーの場合

`C/C++ General` -> `Paths and Symbols` を開き、Languages から `GNU C++` を選択したあと、

- Includes: `プロジェクトルート/tutrcos/include`
- Source Location: `プロジェクトルート/tutrcos/src`

を追加します。

## サンプル

サンプルコードは各クラスのドキュメントに付属しています。

`プロジェクトルート/Core/Src/main_thread.cpp` のようなファイルを作成し、`CMakeLists.txt` の `target_sources` を以下のようにして、`main_thread.cpp` 内にサンプルコードを貼り付ければ動作するはずです(もちろん、CubeMXを用いて各ペリフェラルを適切に設定し、使用するピンやIDなどは各々の環境に合わせる必要があります)。

```cmake
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/main_thread.cpp
)
```

## ライセンス

MIT License

本ライブラリに含まれる、Craig McQueen氏によるCOBS実装(cobs-c)はMIT Licenseに基づきます。
