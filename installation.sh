apt-get update
apt-get install -y python3-dev
apt-get install -y cmake
apt-get install -y g++
apt-get install -y pip
apt-get install libgl1-mesa-glx
apt-get install -y libsm6 libxext6 libxrender-dev
apt-get install -y wget
apt-get install -y libflann1.9 libflann-dev
apt-get install -y libeigen3-dev
apt-get install -y libboost-all-dev
apt-get install -y libvtk6-dev libvtk6.3 libvtk6.3-qt
apt-get install -y 'libqhull*'

pip3 install open3d
pip3 install numpy
pip3 install tqdm

cd pc_cluster/ellipsoidal_clustering
mkdir third_party
cd third_party

git clone https://github.com/pybind/pybind11.git

wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.11.1.tar.gz
tar xvf pcl-1.11.1.tar.gz
cd pcl-pcl-1.11.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
make install

cd ../../../
mkdir build
cd build
cmake ..
make