#include "obstacle_detector/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector()
: Node("obstacle_detector")
{
   // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
   /*************/
}

//Lidarから障害物の情報を取得
void ObstacleDetector::scan_callback()
{
    /***********/
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process()
{
    //scan_obstacle()を呼び出す
   /*************/  
}

//Lidarから障害物情報を取得し，障害物の座標をpublish　※メッセージの型は自分で決めてください
void ObstacleDetector::scan_obstacle()
{
   /************/
}


//無視するlidar情報の範囲の決定(lidarがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan()
{
    /**********/
}