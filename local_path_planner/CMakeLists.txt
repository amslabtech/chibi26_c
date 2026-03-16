cmake_minimum_required(VERSION 3.5)
project(team_local_path_planner)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/local_path_planner.cpp
  src/local_path_planner_node.cpp)
# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()