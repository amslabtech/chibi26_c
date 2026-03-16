cmake_minimum_required(VERSION 3.8)
project(team_localizer)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/localizer.cpp
  src/localizer_node.cpp
  src/odom_model.cpp
  src/particle.cpp
  src/pose.cpp)
# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()