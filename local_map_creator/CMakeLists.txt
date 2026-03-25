cmake_minimum_required(VERSION 3.8)
project(local_map_creator)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/local_map_creator.cpp
  src/local_map_creator_node.cpp
  )
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()