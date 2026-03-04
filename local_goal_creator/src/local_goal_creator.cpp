#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    //pubやsubの定義，tfの統合
    //初期値の設定
    // ループ周期 [Hz]
    // １回で更新するインデックス数
    // グローバルパス内におけるローカルゴールのインデックス
    // 現在位置-ゴール間の距離 [m]
}

void LocalGoalCreator::poseCallback()//subのコールバック関数
{

}

void LocalGoalCreator::pathCallback()//subのコールバック関数
{

}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{

}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
}

void LocalGoalCreator::publishGoal()
{
    //ゴールまでの距離の計算を行う
    //設定値に応じて，ゴール位置の変更を行う
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{

}