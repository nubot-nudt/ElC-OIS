#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <queue>
#include <cmath>
#include <vector>

using namespace std;
namespace py = pybind11;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using KdTree = pcl::search::KdTree<pcl::PointXYZ>;
using KdTreePtr = KdTree::Ptr;
using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;

class ellipsoidalClustering : public pcl::PCLBase<pcl::PointXYZ>
{
  using Base = pcl::PCLBase<pcl::PointXYZ>;
  using Base::deinitCompute;
  using Base::indices_;
  using Base::initCompute;
  using Base::input_;
public:
  ellipsoidalClustering(float rho, float theta, float phi):
    rho_(rho),
    theta_(theta),
    phi_(phi)
  {
  }

  std::array<std::array<int, 2048 >,64 >
  segment()
  {
    std::array<std::array<int, 2048 >,64 > ID_matrix;
    ID_matrix.fill({});
    if (!initCompute() || input_->empty() || indices_->empty())
      return ID_matrix;

    if (!tree_)
      tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    
    tree_->setInputCloud(input_, indices_);

    std::vector<int> L_ID(input_->size(), -1);
    std::vector<bool> extended_flags(input_->size(), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    Graph G;
    std::queue<int> L_Q;
    
    int ID = 0;
    float item_b = tan(theta_*M_PI/180.0f/2.0);
    float item_c = tan(phi_*M_PI/180.0f/2.0);
    float a = rho_/2.0;

    for (auto i_p : *indices_)
    {
      if (extended_flags.at(i_p))
        continue;

      boost::add_edge(ID, ID, G);

      L_Q.push(i_p);
      while (!L_Q.empty())
      {
        auto i_q = L_Q.front();
        L_Q.pop();
        if (extended_flags.at(i_q))
          continue;

        float m = input_->points[i_q].x;
        float n = input_->points[i_q].y;
        float v = input_->points[i_q].z;
        
        float d = sqrt(m*m + n*n);
        float b = item_b * d;
        float c = item_c * d;
        float lambda = atan2(n, m);

        // Find the longest and the shortest ellipsoidal axes
        float max_axis = a > b ? a : b;
        max_axis = max_axis > c ? max_axis : c;

        float min_axis = a < b ? a : b;
        min_axis = min_axis < c ? min_axis : c;
        
        // Find the points that MIGHT be in the ellipsoidal neighbor
        tree_->radiusSearch(i_q, max_axis, nn_indices, nn_distances);

        for (std::size_t i = 0; i < nn_indices.size(); ++i)
        {
          auto i_n = nn_indices.at(i);
          auto q_label = L_ID.at(i_n);

          float x = input_->points[i_n].x;
          float y = input_->points[i_n].y;
          float z = input_->points[i_n].z;

          double item1 = pow((((x-m) * cos(lambda) + (y-n) * sin(lambda)) / a), 2);
          double item2 = pow((((m-x) * sin(lambda) + (y-n) * cos(lambda)) / b), 2);
          double item3 = pow(((z-v) / c), 2);
          
          // Find the points that ARE in the ellipsoidal neighbor
          if (item1 + item2 + item3 > 1.0)
            continue;

          // Mark the IDs of neighboring instances
          if (q_label != -1 && q_label != ID)
            boost::add_edge(ID, q_label, G);

          if (extended_flags.at(i_n))
            continue;

          L_ID.at(i_n) = ID;

          // Stop finding the neighboring points of close points
          if (nn_distances.at(i) <= min_axis)
            extended_flags.at(i_n) = true;
          else
            L_Q.push(i_n);
        }
        extended_flags.at(i_q) = true;
      }

      ID++;
    }

    // Remap instance ID by looking up the graph
    std::vector<int> ID_mapper(boost::num_vertices(G));
    
    auto num_components = boost::connected_components(G, ID_mapper.data());

    for (auto index : *indices_) 
    {
      auto ID = L_ID.at(index);
      auto remapped_ID = ID_mapper.at(ID);

      int row = int(index % 2048);
      int column = int(index / 2048);
      ID_matrix[column][row] = remapped_ID + 1;
    }

    deinitCompute();
    return ID_matrix;
  }

  std::array<std::array<int, 2048 >,64 > ellipsoidalClustering_main(py::array_t<double> input_array_x_,
    py::array_t<double> input_array_y_, py::array_t<double> input_array_z_, int point_num)
  {
    auto input_array_x = input_array_x_.request();
    double *ptr_x = (double *) input_array_x.ptr;
    auto input_array_y = input_array_y_.request();
    double *ptr_y = (double *) input_array_y.ptr;
		auto input_array_z = input_array_z_.request();
		double *ptr_z = (double *) input_array_z.ptr;

    PointCloudPtr cloud(new PointCloud);
    cloud->clear();
    cloud->width = point_num;
    cloud->height = 1;
    cloud->is_dense = false;

    for (int i = 0; i < point_num; i++)
    {
      pcl::PointXYZ point;
      point.x = ptr_x[i];
      point.y = ptr_y[i];
      point.z = ptr_z[i];
      cloud->points.push_back(point);
    }

    KdTreePtr tree(new KdTree);

    input_ = cloud;
    tree_ = tree;

    std::array<std::array<int, 2048 >,64 > ID_matrix = segment();

    cloud->clear();

    return ID_matrix;
  }

private:
  float rho_ = 2.0;
  float theta_ = 2.0;
  float phi_ = 7.5;
  KdTreePtr tree_;
};

PYBIND11_MODULE(ellipsoidalClustering, m) {
    py::class_<ellipsoidalClustering>(m, "ellipsoidalClustering")
    	.def(py::init<float, float, float>())
      .def("ellipsoidalClustering_main", &ellipsoidalClustering::ellipsoidalClustering_main);
}
