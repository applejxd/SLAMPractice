#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <random>

int main() {

    // ハードウェア乱数
    std::random_device seed_gen;
    // メルセンヌ・ツイスター法による擬似乱数生成器
    std::mt19937 engine(seed_gen());
    // 一様実数分布. [0.0, 1.0)の値の範囲で等確率に実数を生成.
    std::uniform_real_distribution<> dist(0.0, 1.0);

    // 検索対象の点群
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // unorganized (2次元整列しない) 点群設定. 点の数は 1000x1 個
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // 点生成
    for (std::size_t i = 0; i < cloud->size(); ++i) {
        (*cloud)[i].x = static_cast<float>(1024.0f * dist(engine));
        (*cloud)[i].y = static_cast<float>(1024.0f * dist(engine));
        (*cloud)[i].z = static_cast<float>(1024.0f * dist(engine));
    }

    // FLANN 実装による kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kd-tree に点群登録
    kdtree.setInputCloud(cloud);

    // 検索の query となる点
    pcl::PointXYZ searchPoint;
    searchPoint.x = static_cast<float>(1024.0f * dist(engine));
    searchPoint.y = static_cast<float>(1024.0f * dist(engine));
    searchPoint.z = static_cast<float>(1024.0f * dist(engine));

    // K nearest neighbor search
    {
        // 検索数
        uint16_t K = 10;

        std::cout << "K nearest neighbor search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with K=" << K << std::endl;

        // 検索結果のインデックスリスト
        std::vector<int> pointIdxKNNSearch(K);
        // 検索結果の距離リスト
        std::vector<float> pointKNNSquaredDistance(K);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
            // 結果を表示
            for (const auto& index: pointIdxKNNSearch) {
                std::cout << "    " << (*cloud)[static_cast<size_t>(index)].x
                          << " " << (*cloud)[static_cast<size_t>(index)].y
                          << " " << (*cloud)[static_cast<size_t>(index)].z
                          << " (squared distance: " << index << ")" << std::endl;
            }
        }
    }

    // Neighbors within radius search
    {
        // 検索半径
        auto radius = static_cast<float>(256.0f * dist(engine));;

        std::cout << "Neighbors within radius search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with radius=" << radius << std::endl;

        // 検索結果のインデックスリスト
        std::vector<int> pointIdxRadiusSearch;
        // 検索結果の距離リスト
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // 結果を表示
            for (const auto &index: pointIdxRadiusSearch) {
                std::cout << "    " << (*cloud)[static_cast<size_t>(index)].x
                          << " " << (*cloud)[static_cast<size_t>(index)].y
                          << " " << (*cloud)[static_cast<size_t>(index)].z
                          << " (squared distance: " << index << ")" << std::endl;
            }
        }
    }

    return 0;
}